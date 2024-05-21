TABLE_QSS = '''
    QTableWidget {
        color: rgb(200, 200, 200);
        background-color: rgb(39, 44, 54);
        padding: 10px;
        border-radius: 5px;
        gridline-color: rgb(54, 57, 72);
        border-bottom: 1px solid rgb(54, 57, 72);
    }

    QTableWidget::item {
        border-color: rgb(54, 57, 72);
        padding-left: 5px;
        padding-right: 5px;
        gridline-color: rgb(54, 57, 72);
    }

    QTableWidget::item:selected {
        background-color: rgb(115, 121, 145);
    }

    QScrollBar:horizontal {
        border: none;
        background: rgb(52, 59, 72);
        height: 14px;
        margin: 0px 21px 0 21px;
        border-radius: 0px;
    }

    QScrollBar:vertical {
        border: none;
        background: rgb(52, 59, 72);
        width: 14px;
        margin: 21px 0 21px 0;
        border-radius: 0px;
    }

    QHeaderView {
        qproperty-defaultAlignment: AlignCenter;
        color : rgb(255, 255, 255);
        background-color: rgb(39, 44, 54);
    }

    QHeaderView::section {
        Background-color: rgb(39, 44, 54);
        max-width: 30px;
        border: 1px solid rgb(54, 57, 72);
        border-style: none;
        border-bottom: 1px solid rgb(54, 57, 72);
        border-right: 1px solid rgb(54, 57, 72);
    }

    QTableWidget::horizontalHeader {
        background-color: rgb(81, 255, 0);
    }

    QHeaderView::section:horizontal {
        border: 1px solid rgb(32, 34, 42);
        background-color: rgb(27, 29, 35);
        padding: 3px;
        border-top-left-radius: 7px;
        border-top-right-radius: 7px;
    }

    QHeaderView::section:vertical {
        border: 1px solid rgb(54, 57, 72);
    }

    /* SCROLL BARS */

    QScrollBar:horizontal {
        border: none;
        background: rgb(52, 59, 72);
        height: 14px;
        margin: 0px 21px 0 21px;
        border-radius: 0px;
    }

    QScrollBar::handle:horizontal {
        background: rgb(85, 170, 255);
        min-width: 25px;
        border-radius: 7px;
    }

    QScrollBar::add-line:horizontal {
        border: none;
        background: rgb(55, 63, 77);
        width: 20px;
        border-top-right-radius: 7px;
        border-bottom-right-radius: 7px;
        subcontrol-position: right;
        subcontrol-origin: margin;
    }

    QScrollBar::sub-line:horizontal {
        border: none;
        background: rgb(55, 63, 77);
        width: 20px;
        border-top-left-radius: 7px;
        border-bottom-left-radius: 7px;
        subcontrol-position: left;
        subcontrol-origin: margin;
    }

    QScrollBar::up-arrow:horizontal, QScrollBar::down-arrow:horizontal {
        background: none;
    }

    QScrollBar::add-page:horizontal, QScrollBar::sub-page:horizontal {
        background: none;
    }

    QScrollBar:vertical {
        border: none;
        background: rgb(52, 59, 72);
        width: 14px;
        margin: 21px 0 21px 0;
        border-radius: 0px;
    }

    QScrollBar::handle:vertical {
        background: rgb(85, 170, 255);
        min-height: 25px;
        border-radius: 7px;
    }

    QScrollBar::add-line:vertical {
        border: none;
        background: rgb(55, 63, 77);
        height: 20px;
        border-bottom-left-radius: 7px;
        border-bottom-right-radius: 7px;
        subcontrol-position: bottom;
        subcontrol-origin: margin;
    }

    QScrollBar::sub-line:vertical {
        border: none;
        background: rgb(55, 63, 77);
        height: 20px;
        border-top-left-radius: 7px;
        border-top-right-radius: 7px;
        subcontrol-position: top;
        subcontrol-origin: margin;
    }

    QScrollBar::up-arrow:vertical, QScrollBar::down-arrow:vertical {
        background: none;
    }

    QScrollBar::add-page:vertical, QScrollBar::sub-page:vertical  {
        background: none;
    } '''

BTN_QSS = '''
            QPushButton:hover {
                background-color: qlineargradient(spread:pad, x1:0, y1:0, x2:0, y2:1,   stop:0 rgba(40, 166, 162, 255), stop:1 rgba(60, 186, 162, 255))
            }
            QPushButton {
                border : 2px solid rgb(29, 233, 182);
                background-color: rgb(49, 54, 59);
                color : rgb(29, 233, 182);
                border-radius: 10px;
            }

            QPushButton:disabled {
                background-color: rgb(45, 90, 83);
                color : rgb(86, 170, 156);
            }'''
