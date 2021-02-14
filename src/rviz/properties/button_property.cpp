/*
 * Copyright (c) 2012, Willow Garage, Inc.
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

#include <rviz/properties/button_property.h>
#include <rviz/properties/button_editor.h>
#include <QApplication>

namespace rviz
{
ButtonProperty::ButtonProperty(const QString& name,
                               const QString& description,
                               Property* parent,
                               const char* changed_slot,
                               QObject* receiver)
  : Property(name, "", description, parent, changed_slot, receiver)
{
}

bool ButtonProperty::setValue(const QVariant& new_value)
{
  Q_EMIT aboutToChange();
  Q_EMIT changed();
  if (model_)
  {
    model_->emitDataChanged(this);
  }
  return true;
}

bool ButtonProperty::paint(QPainter* painter, const QStyleOptionViewItem& option) const
{
  painter->save();

  if (!(getViewFlags(0) & Qt::ItemIsEnabled))
  {
    painter->setPen(QColor(Qt::lightGray));
  }

  QRect rect = option.rect;
  ButtonEditor::paintButton(painter, rect);

//  QStyleOptionButton button;
//  button.state = QStyle::State_Active;
//  if (!(getViewFlags(0) & Qt::ItemIsEnabled))
//  {
//    button.state |= static_cast<QStyle::StateFlag>(~QStyle::State_Enabled);
//  }
//  else
//  {
//    button.state |= QStyle::State_Enabled;
//  }
//
//  button.rect = QRect(option.rect.left() + 0, option.rect.top() + 0, 20, 20);
//
//  if (button.rect.contains(mouse_point_))
//  {
//    if (mouse_event_type_ == 0)
//    {
//      button.state |= QStyle::State_MouseOver;
//    }
//    else if (mouse_event_type_ == 1)
//    {
//      button.state |= QStyle::State_Sunken;
//    }
//  }
//  QApplication::style()->drawControl(QStyle::CE_PushButton, &button, painter);

  painter->restore();

  return true; // return true, since this function has done the painting.
}

QWidget* ButtonProperty::createEditor(QWidget* parent, const QStyleOptionViewItem& /*option*/)
{
  // called once the user clicked on the button
  auto* reset_button = new ButtonEditor(parent);
  return reset_button;
}

bool ButtonProperty::editorEvent(QEvent* event,
                                 QAbstractItemModel* model,
                                 const QStyleOptionViewItem& option,
                                 const QModelIndex& index)
{
  auto* mouse_event = static_cast<QMouseEvent*>(event);
  mouse_point_ = mouse_event->pos();
  mouse_event_type_ = -1;

  switch (mouse_event->type())
  {
  case QEvent::MouseMove:
  {
    // FIXME: this isn't working
    QApplication::setOverrideCursor(Qt::PointingHandCursor);
    mouse_event_type_ = 0;
    return true;
  }
  case QEvent::MouseButtonPress:
  {
    mouse_event_type_ = 1;
    return true;
  }
  case QEvent::MouseButtonRelease:
  {
    return true;
  }
  }

  return false;
}

} // end namespace rviz
