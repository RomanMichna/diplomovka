/****************************************************************************
**
** Copyright (C) 2010 Nokia Corporation and/or its subsidiary(-ies).
** All rights reserved.
** Contact: Nokia Corporation (qt-info@nokia.com)
**
** This file is part of the Symbian application wrapper of the Qt Toolkit.
**
** $QT_BEGIN_LICENSE:LGPL$
** Commercial Usage
** Licensees holding valid Qt Commercial licenses may use this file in
** accordance with the Qt Commercial License Agreement provided with the
** Software or, alternatively, in accordance with the terms contained in
** a written agreement between you and Nokia.
**
** GNU Lesser General Public License Usage
** Alternatively, this file may be used under the terms of the GNU Lesser
** General Public License version 2.1 as published by the Free Software
** Foundation and appearing in the file LICENSE.LGPL included in the
** packaging of this file.  Please review the following information to
** ensure the GNU Lesser General Public License version 2.1 requirements
** will be met: http://www.gnu.org/licenses/old-licenses/lgpl-2.1.html.
**
** In addition, as a special exception, Nokia gives you certain additional
** rights.  These rights are described in the Nokia Qt LGPL Exception
** version 1.1, included in the file LGPL_EXCEPTION.txt in this package.
**
** GNU General Public License Usage
** Alternatively, this file may be used under the terms of the GNU
** General Public License version 3.0 as published by the Free Software
** Foundation and appearing in the file LICENSE.GPL included in the
** packaging of this file.  Please review the following information to
** ensure the GNU General Public License version 3.0 requirements will be
** met: http://www.gnu.org/copyleft/gpl.html.
**
** If you have questions regarding the use of this file, please contact
** Nokia at qt-info@nokia.com.
** $QT_END_LICENSE$
**
****************************************************************************/

#ifndef QS60MAINAPPUI_H
#define QS60MAINAPPUI_H

#include <QtCore/qglobal.h>

#ifdef Q_WS_S60

#include <aknappui.h>

QT_BEGIN_HEADER

QT_BEGIN_NAMESPACE

QT_MODULE(Gui)

class Q_GUI_EXPORT QS60MainAppUi : public CAknAppUi
{
public:
    QS60MainAppUi();
    // The virtuals are for qdoc.
    virtual ~QS60MainAppUi();

    virtual void ConstructL();

    virtual void RestoreMenuL(CCoeControl* menuWindow,TInt resourceId,TMenuType menuType);
    virtual void DynInitMenuBarL(TInt resourceId, CEikMenuBar *menuBar);
    virtual void DynInitMenuPaneL(TInt resourceId, CEikMenuPane *menuPane);

    virtual void HandleCommandL( TInt command );

    virtual void HandleResourceChangeL(TInt type);

    virtual void HandleStatusPaneSizeChange();

protected:
    virtual void HandleWsEventL(const TWsEvent& event, CCoeControl* destination);
};

QT_END_NAMESPACE

QT_END_HEADER

#endif // Q_WS_S60

#endif // QS60MAINAPPUI_H
