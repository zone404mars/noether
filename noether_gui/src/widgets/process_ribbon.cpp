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

#include <noether_gui/widgets/process_ribbon.h>

#include <QFrame>
#include <QHBoxLayout>
#include <QIcon>
#include <QStackedWidget>
#include <QTabBar>
#include <QToolButton>
#include <QVBoxLayout>

namespace
{
/** @brief Cote des icones des outils, en pixels */
const int kIconSize = 28;
/** @brief Largeur minimale d'un bouton d'outil, pour que les libelles restent lisibles */
const int kToolMinimumWidth = 84;
}  // namespace

namespace noether
{
ProcessRibbon::ProcessRibbon(Translator& translator, QWidget* parent)
  : QWidget(parent), translator_(translator), tab_bar_(new QTabBar(this)), tool_rows_(new QStackedWidget(this))
{
  tab_bar_->setExpanding(false);
  tab_bar_->setDrawBase(true);
  auto* layout = new QVBoxLayout(this);
  layout->setContentsMargins(6, 4, 6, 0);
  layout->setSpacing(0);
  layout->addWidget(tab_bar_);
  layout->addWidget(tool_rows_);
  connect(tab_bar_, &QTabBar::currentChanged, this, [this](int tab) {
    tool_rows_->setCurrentIndex(tab);
    if (on_tab_changed)
    {
      on_tab_changed(tab);
    }
  });
}

int ProcessRibbon::addTab(const Bilingual& label)
{
  auto* row = new QWidget(tool_rows_);
  auto* row_layout = new QHBoxLayout(row);
  row_layout->setContentsMargins(4, 4, 4, 4);
  row_layout->setSpacing(4);
  row_layout->addStretch();
  tool_rows_->addWidget(row);
  row_layouts_.push_back(row_layout);
  const int tab = tab_bar_->addTab(QString());
  translator_.bind([this, tab, label]() { tab_bar_->setTabText(tab, translator_.text(label)); });
  return tab;
}

QToolButton* ProcessRibbon::addTool(const int tab, const QString& icon, const Bilingual& label, const Bilingual& tip)
{
  QHBoxLayout* row_layout = row_layouts_.at(static_cast<std::size_t>(tab));
  auto* button = new QToolButton(row_layout->parentWidget());
  button->setIcon(QIcon(icon));
  button->setIconSize(QSize(kIconSize, kIconSize));
  button->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);
  button->setAutoRaise(true);
  button->setMinimumWidth(kToolMinimumWidth);
  translator_.bindText(button, label);
  translator_.bindToolTip(button, tip);
  // Le dernier element de la rangee est l'espace extensible : les outils s'inserent avant lui
  row_layout->insertWidget(row_layout->count() - 1, button);
  return button;
}

void ProcessRibbon::addSeparator(const int tab)
{
  QHBoxLayout* row_layout = row_layouts_.at(static_cast<std::size_t>(tab));
  auto* line = new QFrame(row_layout->parentWidget());
  line->setFrameShape(QFrame::VLine);
  line->setFrameShadow(QFrame::Sunken);
  row_layout->insertWidget(row_layout->count() - 1, line);
}

int ProcessRibbon::currentTab() const { return tab_bar_->currentIndex(); }

void ProcessRibbon::setCurrentTab(const int tab) { tab_bar_->setCurrentIndex(tab); }

}  // namespace noether
