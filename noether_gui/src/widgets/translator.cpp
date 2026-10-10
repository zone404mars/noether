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

#include <noether_gui/widgets/translator.h>

#include <QAbstractButton>
#include <QAction>
#include <QDockWidget>
#include <QLabel>
#include <QMenu>
#include <QSettings>

namespace
{
/** @brief Cle de la langue dans les reglages de l'application */
const char* const kLanguageKey = "language";
}  // namespace

namespace noether
{
Translator::Translator(const Language language) : language_(language) {}

void Translator::setLanguage(const Language language)
{
  language_ = language;
  for (const std::function<void()>& apply : bindings_)
  {
    apply();
  }
}

QString Translator::text(const Bilingual& text) const
{
  return language_ == Language::FRENCH ? text.french : text.english;
}

void Translator::bind(const std::function<void()>& apply)
{
  apply();
  bindings_.push_back(apply);
}

void Translator::bindText(QAbstractButton* button, const Bilingual& text)
{
  bind([this, button, text]() { button->setText(this->text(text)); });
}

void Translator::bindText(QLabel* label, const Bilingual& text)
{
  bind([this, label, text]() { label->setText(this->text(text)); });
}

void Translator::bindText(QAction* action, const Bilingual& text)
{
  bind([this, action, text]() { action->setText(this->text(text)); });
}

void Translator::bindText(QMenu* menu, const Bilingual& text)
{
  bind([this, menu, text]() { menu->setTitle(this->text(text)); });
}

void Translator::bindText(QDockWidget* dock, const Bilingual& text)
{
  bind([this, dock, text]() { dock->setWindowTitle(this->text(text)); });
}

void Translator::bindToolTip(QWidget* widget, const Bilingual& text)
{
  bind([this, widget, text]() { widget->setToolTip(this->text(text)); });
}

Language Translator::savedLanguage()
{
  return QSettings().value(kLanguageKey, "fr").toString() == "en" ? Language::ENGLISH : Language::FRENCH;
}

void Translator::saveLanguage(const Language language)
{
  QSettings().setValue(kLanguageKey, language == Language::ENGLISH ? "en" : "fr");
}

}  // namespace noether
