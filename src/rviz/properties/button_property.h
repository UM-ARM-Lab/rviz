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
#ifndef BUTTON_PROPERTY_H
#define BUTTON_PROPERTY_H

#include <QAbstractItemModel>
#include <QEvent>
#include <QMouseEvent>
#include <QPainter>
#include <QPoint>
#include <QPushButton>
#include <QStyle>

#include <rviz/properties/property.h>
#include <rviz/properties/property_tree_model.h>
#include <rviz/rviz_export.h>

namespace rviz
{
/** @brief Property specialized to provide getter for buttons? */
class RVIZ_EXPORT ButtonProperty : public Property
{
  Q_OBJECT
public:
  ButtonProperty(const QString& name = QString(),
                 const QString& description = QString(),
                 Property* parent = nullptr,
                 const char* changed_slot = nullptr,
                 QObject* receiver = nullptr);

  bool setValue(const QVariant& new_value) override;

  bool paint(QPainter* painter, const QStyleOptionViewItem& option) const override;

  QWidget* createEditor(QWidget* parent, const QStyleOptionViewItem& option) override;

  bool editorEvent(QEvent* event,
                   QAbstractItemModel* model,
                   const QStyleOptionViewItem& option,
                   const QModelIndex& index);
private:
  QPoint mouse_point_;
  int mouse_event_type_;
};

} // end namespace rviz

#endif // BUTTON_PROPERTY_H
