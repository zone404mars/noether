/*
 * Copyright 2026 Zone 404
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <noether_cad/step_reader.h>

#include <BRepAdaptor_Surface.hxx>
#include <BRepBuilderAPI_Transform.hxx>
#include <BRepGProp.hxx>
#include <BRepMesh_IncrementalMesh.hxx>
#include <BRep_Builder.hxx>
#include <BRep_Tool.hxx>
#include <GProp_GProps.hxx>
#include <Interface_Static.hxx>
#include <Poly_Triangulation.hxx>
#include <STEPControl_Reader.hxx>
#include <TopExp_Explorer.hxx>
#include <TopLoc_Location.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Compound.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Shape.hxx>
#include <algorithm>
#include <cmath>
#include <numeric>
#include <sstream>
#include <unordered_map>

namespace
{
using noether::cad::CadModel;
using noether::cad::Face;
using noether::cad::FaceType;
using noether::cad::Mesh;
using noether::cad::SolidSelection;
using noether::cad::StepError;

/**
 * @brief Tolerance de fusion des sommets, en metres
 * @details OpenCASCADE maille face par face : deux faces voisines produisent chacune leurs
 * sommets le long de l'arete commune. Sans fusion, le maillage n'a aucune connectivite.
 * 0,1 micron est au-dessus du bruit numerique et en dessous de toute caracteristique reelle.
 */
constexpr double kWeldTolerance = 1e-7;

/** @brief Point OpenCASCADE en vecteur Eigen */
Eigen::Vector3d toEigen(const gp_XYZ& point) { return Eigen::Vector3d(point.X(), point.Y(), point.Z()); }

/**
 * @brief Charge la geometrie d'un STEP, exprimee en metres
 * @param declared_unit Unite retenue pour interpreter les nombres du fichier
 * @throws StepError si la lecture ou le transfert echoue
 */
TopoDS_Shape loadShape(const std::string& path, const std::string& declared_unit)
{
  // OpenCASCADE ramene la geometrie a son unite de travail ; elle est fixee au millimetre pour
  // savoir dans quelle unite la forme arrive, la conversion en metres est faite ensuite
  Interface_Static::SetCVal("xstep.cascade.unit", "MM");
  STEPControl_Reader reader;
  if (reader.ReadFile(path.c_str()) != IFSelect_RetDone)
  {
    throw StepError("Lecture STEP echouee : " + path);
  }
  const TopoDS_Shape shape = (reader.TransferRoots() > 0) ? reader.OneShape() : TopoDS_Shape();
  if (shape.IsNull())
  {
    throw StepError("Aucune geometrie transferee depuis " + path);
  }
  // La forme est en millimetres apres application de l'unite du fichier : on revient aux nombres
  // bruts du fichier, puis on applique l'unite retenue, qui peut avoir ete imposee. L'unite du
  // fichier doit donc etre lisible meme quand une autre est imposee
  const double scale = noether::cad::metersPerUnit(declared_unit) * 1e-3 /
                       noether::cad::metersPerUnit(noether::cad::readDeclaredUnit(path));
  gp_Trsf scaling;
  scaling.SetScale(gp_Pnt(0.0, 0.0, 0.0), scale);
  return BRepBuilderAPI_Transform(shape, scaling, Standard_True).Shape();
}

/** @brief Type analytique d'une face, tel qu'OpenCASCADE le classe */
FaceType classify(const TopoDS_Face& face)
{
  switch (BRepAdaptor_Surface(face, Standard_True).GetType())
  {
    case GeomAbs_Plane:
      return FaceType::PLANE;
    case GeomAbs_Cylinder:
      return FaceType::CYLINDER;
    case GeomAbs_Cone:
      return FaceType::CONE;
    case GeomAbs_Sphere:
      return FaceType::SPHERE;
    case GeomAbs_Torus:
      return FaceType::TORUS;
    case GeomAbs_BSplineSurface:
    case GeomAbs_BezierSurface:
      return FaceType::BSPLINE;
    default:
      return FaceType::OTHER;
  }
}

/** @brief Normale sortante d'un plan : elle suit l'orientation de la face, pas celle de la surface */
Eigen::Vector3d outwardPlaneNormal(const TopoDS_Face& face, const BRepAdaptor_Surface& surface)
{
  const Eigen::Vector3d normal = toEigen(surface.Plane().Axis().Direction().XYZ());
  return (face.Orientation() == TopAbs_REVERSED) ? Eigen::Vector3d(-normal) : normal;
}

/** @brief Point et direction d'un axe OpenCASCADE, ecrits dans la face */
void setAxis(const gp_Ax1& axis, Face& out)
{
  out.axis_point = toEigen(axis.Location().XYZ());
  out.axis_direction = toEigen(axis.Direction().XYZ());
}

/** @brief Parametres analytiques exacts d'une face selon son type : normale, axe, rayon */
void fillAnalyticParameters(const TopoDS_Face& face, Face& out)
{
  const BRepAdaptor_Surface surface(face, Standard_True);
  switch (out.type)
  {
    case FaceType::PLANE:
      out.normal = outwardPlaneNormal(face, surface);
      break;
    case FaceType::CYLINDER:
      setAxis(surface.Cylinder().Axis(), out);
      out.radius = surface.Cylinder().Radius();
      break;
    case FaceType::CONE:
      setAxis(surface.Cone().Axis(), out);
      break;
    case FaceType::SPHERE:
      out.axis_point = toEigen(surface.Sphere().Location().XYZ());
      out.radius = surface.Sphere().Radius();
      break;
    case FaceType::TORUS:
      setAxis(surface.Torus().Axis(), out);
      out.radius = surface.Torus().MajorRadius();
      break;
    default:
      break;
  }
}

/** @brief Aire et centre de gravite de la surface exacte d'une face */
void fillSurfaceProperties(const TopoDS_Face& face, Face& out)
{
  GProp_GProps properties;
  BRepGProp::SurfaceProperties(face, properties);
  out.area = properties.Mass();
  out.centroid = toEigen(properties.CentreOfMass().XYZ());
}

/** @brief Cle de fusion d'un sommet : coordonnees quantifiees a kWeldTolerance */
struct VertexKey
{
  /** @brief Coordonnees quantifiees */
  long long x;
  long long y;
  long long z;

  bool operator==(const VertexKey& other) const { return x == other.x && y == other.y && z == other.z; }
};

/** @brief Hachage d'une cle de fusion : melange de trois entiers */
struct VertexKeyHash
{
  std::size_t operator()(const VertexKey& key) const
  {
    std::size_t seed = static_cast<std::size_t>(key.x) * 73856093ULL;
    seed ^= static_cast<std::size_t>(key.y) * 19349663ULL;
    seed ^= static_cast<std::size_t>(key.z) * 83492791ULL;
    return seed;
  }
};

/** @brief Table de fusion : cle quantifiee vers indice du sommet dans le maillage */
using WeldTable = std::unordered_map<VertexKey, int, VertexKeyHash>;

/** @brief Indice du sommet du maillage a cette position, cree s'il n'existe pas encore */
int weldVertex(const Eigen::Vector3d& vertex, WeldTable& index_of, Mesh& mesh)
{
  const VertexKey key{ std::llround(vertex.x() / kWeldTolerance),
                       std::llround(vertex.y() / kWeldTolerance),
                       std::llround(vertex.z() / kWeldTolerance) };
  const auto found = index_of.find(key);
  if (found != index_of.end())
  {
    return found->second;
  }
  const int index = static_cast<int>(mesh.vertices.size());
  mesh.vertices.push_back(vertex);
  index_of.emplace(key, index);
  return index;
}

/** @brief Ajoute au maillage les triangles d'une face deja maillee, sommets fusionnes */
void appendFaceTriangles(const TopoDS_Face& face, WeldTable& index_of, Mesh& mesh)
{
  TopLoc_Location location;
  const Handle(Poly_Triangulation) triangulation = BRep_Tool::Triangulation(face, location);
  if (triangulation.IsNull())
  {
    return;
  }
  // Les noeuds d'OpenCASCADE sont numerotes a partir de 1
  std::vector<int> local_to_global(static_cast<std::size_t>(triangulation->NbNodes()) + 1, -1);
  for (int node = 1; node <= triangulation->NbNodes(); ++node)
  {
    const gp_Pnt point = triangulation->Node(node).Transformed(location.Transformation());
    local_to_global[static_cast<std::size_t>(node)] = weldVertex(toEigen(point.XYZ()), index_of, mesh);
  }
  // Une face inversee inverse le sens de ses triangles, pour que la normale reste sortante
  const bool reversed = (face.Orientation() == TopAbs_REVERSED);
  for (int index = 1; index <= triangulation->NbTriangles(); ++index)
  {
    int a = 0;
    int b = 0;
    int c = 0;
    triangulation->Triangle(index).Get(a, b, c);
    if (reversed)
    {
      std::swap(b, c);
    }
    mesh.triangles.emplace_back(local_to_global[a], local_to_global[b], local_to_global[c]);
  }
}

/** @brief Description courte de chaque solide de la forme, dans l'ordre de parcours */
std::vector<std::string> describeSolids(const TopoDS_Shape& shape)
{
  std::vector<std::string> solids;
  for (TopExp_Explorer solid(shape, TopAbs_SOLID); solid.More(); solid.Next())
  {
    int faces = 0;
    for (TopExp_Explorer face(solid.Current(), TopAbs_FACE); face.More(); face.Next())
    {
      ++faces;
    }
    std::ostringstream text;
    text << "solide " << solids.size() << " (" << faces << " faces)";
    solids.push_back(text.str());
  }
  return solids;
}

/** @brief Refuse une selection qui cite un solide inexistant */
void checkSolidIndices(const SolidSelection& selection, const int count)
{
  for (const std::vector<int>* list : { &selection.kept, &selection.excluded })
  {
    for (const int index : *list)
    {
      if (index < 0 || index >= count)
      {
        throw StepError("Solide " + std::to_string(index) + " inexistant : le fichier en compte " +
                        std::to_string(count) + ", numerotes de 0 a " + std::to_string(count - 1));
      }
    }
  }
}

/** @brief Refus d'un fichier a plusieurs solides sans selection, avec la liste des solides */
StepError ambiguousSolids(const std::vector<std::string>& solids)
{
  // Choisir a la place de l'operateur pourrait faire poncer la mauvaise partie du fichier
  std::ostringstream text;
  text << "Le fichier contient " << solids.size()
       << " solides : choisissez de tous les mailler, ou lesquels mailler :";
  for (const std::string& solid : solids)
  {
    text << "\n  " << solid;
  }
  return StepError(text.str());
}

/** @brief Solides demandes avant exclusion : tous, la liste triee sans doublon, ou le seul solide */
std::vector<int> requestedSolids(const SolidSelection& selection, const std::vector<std::string>& solids)
{
  const int count = static_cast<int>(solids.size());
  std::vector<int> requested = selection.kept;
  if (selection.all)
  {
    requested.resize(static_cast<std::size_t>(count));
    std::iota(requested.begin(), requested.end(), 0);
  }
  else if (requested.empty() && count == 1)
  {
    requested.push_back(0);
  }
  else if (requested.empty())
  {
    throw ambiguousSolids(solids);
  }
  // Un indice cite deux fois ne doit compter qu'une fois
  std::sort(requested.begin(), requested.end());
  requested.erase(std::unique(requested.begin(), requested.end()), requested.end());
  return requested;
}

/** @brief Indices des solides a mailler, controles contre ceux du fichier, exclusions retirees */
std::vector<int> retainedSolids(const SolidSelection& selection, const std::vector<std::string>& solids)
{
  checkSolidIndices(selection, static_cast<int>(solids.size()));
  std::vector<int> retained = requestedSolids(selection, solids);
  for (const int excluded : selection.excluded)
  {
    retained.erase(std::remove(retained.begin(), retained.end(), excluded), retained.end());
  }
  if (retained.empty())
  {
    throw StepError("La selection de solides ne retient rien");
  }
  return retained;
}

/** @brief Forme des seuls solides retenus ; la forme d'origine quand tout est retenu */
TopoDS_Shape shapeOfSolids(const TopoDS_Shape& shape, const std::vector<int>& retained, const int total)
{
  // Tout retenu : la forme d'origine garde exactement le meme parcours des faces
  if (static_cast<int>(retained.size()) == total)
  {
    return shape;
  }
  BRep_Builder builder;
  TopoDS_Compound compound;
  builder.MakeCompound(compound);
  int index = 0;
  for (TopExp_Explorer solid(shape, TopAbs_SOLID); solid.More(); solid.Next(), ++index)
  {
    if (std::find(retained.begin(), retained.end(), index) != retained.end())
    {
      builder.Add(compound, solid.Current());
    }
  }
  return compound;
}

/** @brief Applique la selection de solides et note dans le modele ce qui est retenu et ecarte */
TopoDS_Shape selectSolids(const TopoDS_Shape& loaded, const SolidSelection& selection, CadModel& model)
{
  const std::vector<std::string> solids = describeSolids(loaded);
  model.solid_total = static_cast<int>(solids.size());
  model.retained_solids = retainedSolids(selection, solids);
  model.solid_count = static_cast<int>(model.retained_solids.size());
  for (int index = 0; index < model.solid_total; ++index)
  {
    const auto& retained = model.retained_solids;
    if (std::find(retained.begin(), retained.end(), index) == retained.end())
    {
      model.excluded_solids.push_back(index);
    }
  }
  return shapeOfSolids(loaded, model.retained_solids, model.solid_total);
}

/** @brief Maille la forme et remplit le maillage et la table des faces du modele */
void tessellate(const TopoDS_Shape& shape, CadModel& model)
{
  // Les deflexions sont en unites du modele, donc en metres ici
  BRepMesh_IncrementalMesh mesher(
      shape, model.tessellation.linear_deflection, Standard_False, model.tessellation.angular_deflection, Standard_True);
  mesher.Perform();
  WeldTable index_of;
  int face_id = 0;
  for (TopExp_Explorer explorer(shape, TopAbs_FACE); explorer.More(); explorer.Next(), ++face_id)
  {
    const TopoDS_Face face = TopoDS::Face(explorer.Current());
    Face description;
    description.id = face_id;
    description.type = classify(face);
    description.triangle_begin = static_cast<int>(model.mesh.triangles.size());
    appendFaceTriangles(face, index_of, model.mesh);
    description.triangle_end = static_cast<int>(model.mesh.triangles.size());
    fillAnalyticParameters(face, description);
    fillSurfaceProperties(face, description);
    model.faces.push_back(description);
  }
}

/** @brief Translation du modele vers le centre de sa boite englobante, notee dans T_source_output */
void recenterOnBoundingBox(CadModel& model)
{
  // Translation pure : une remise en place avec rotation ne se verifierait pas a la main
  const auto box = model.boundingBox();
  const Eigen::Vector3d offset = -0.5 * (box.first + box.second);
  for (Eigen::Vector3d& vertex : model.mesh.vertices)
  {
    vertex += offset;
  }
  for (Face& face : model.faces)
  {
    face.centroid += offset;
    // Le point d'axe n'a de sens que pour les faces qui en portent un ; une direction est invariante
    if (face.axis_direction.norm() > 0.0 || face.type == FaceType::SPHERE)
    {
      face.axis_point += offset;
    }
  }
  model.T_source_output.translation() = offset;
}
}  // namespace

namespace noether
{
namespace cad
{
StepInfo readInfo(const std::string& path, const std::string& forced_unit)
{
  StepInfo info;
  info.source = path;
  info.declared_unit = forced_unit.empty() ? readDeclaredUnit(path) : forced_unit;
  const TopoDS_Shape shape = loadShape(path, info.declared_unit);
  info.solids = describeSolids(shape);
  for (TopExp_Explorer explorer(shape, TopAbs_FACE); explorer.More(); explorer.Next())
  {
    ++info.face_count;
    ++info.faces_by_type[classify(TopoDS::Face(explorer.Current()))];
  }
  return info;
}

CadModel buildModel(const std::string& path,
                    const TessellationSettings& settings,
                    const Recentering recentering,
                    const std::string& forced_unit,
                    const SolidSelection& selection)
{
  const auto valid = [](const double deflection) { return std::isfinite(deflection) && deflection > 0.0; };
  if (!valid(settings.linear_deflection) || !valid(settings.angular_deflection))
  {
    throw StepError("Les deflexions de tessellation doivent etre des nombres finis strictement positifs");
  }
  CadModel model;
  model.source = path;
  model.unit_forced = !forced_unit.empty();
  model.declared_unit = forced_unit.empty() ? readDeclaredUnit(path) : forced_unit;
  model.tessellation = settings;
  // Tout l'assemblage garde la geometrie de collision complete ; un solide ecarte disparait du
  // maillage, de l'ecran et de la collision
  tessellate(selectSolids(loadShape(path, model.declared_unit), selection, model), model);
  if (model.mesh.triangles.empty())
  {
    throw StepError("La tessellation n'a produit aucun triangle : deflexions trop grandes ?");
  }
  if (recentering == Recentering::BOUNDING_BOX)
  {
    recenterOnBoundingBox(model);
  }
  return model;
}

}  // namespace cad
}  // namespace noether
