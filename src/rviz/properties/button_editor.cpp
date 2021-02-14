/*
 * Copyright (c) 2011, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <QMetaObject>
#include <QMetaProperty>

#include <QPainter>

#include <rviz/load_resource.h>
#include <rviz/properties/button_editor.h>

static constexpr auto icon_name = "package://rviz/icons/reset.svg";

namespace rviz
{
ButtonEditor::ButtonEditor(QWidget* parent)
  : QPushButton(parent)
{
}

void ButtonEditor::paintEvent(QPaintEvent* event)
{
//  QPainter painter(this);
//  painter.setPen(Qt::black);
//  paintButton(&painter, rect());
}

void ButtonEditor::paintButton(QPainter* painter, const QRect& rect)
{
  painter->save();

  int size = rect.height();

  QStyleOptionButton button;
  button.state = QStyle::State_Active;
  button.rect = QRect(rect.left(), rect.top(), size, size);
  button.icon = loadPixmap(icon_name);
  QApplication::style()->drawControl(QStyle::CE_PushButton, &button, painter);

  painter->restore();
}

} // end namespace rviz
