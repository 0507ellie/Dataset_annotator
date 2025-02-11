import time
from qtpy import QtCore, QtGui, QtWidgets

class Loader(QtWidgets.QWidget):
    def __init__(self, text='Loading...', parent=None):
        super().__init__(parent)
        self.text = text
        self.gradient = QtGui.QConicalGradient(.5, .5, 0)
        self.gradient.setCoordinateMode(self.gradient.ObjectBoundingMode)
        self.gradient.setColorAt(.25, QtCore.Qt.transparent)
        self.gradient.setColorAt(.75, QtCore.Qt.transparent)

        self.animation = QtCore.QVariantAnimation(
            startValue=0., endValue=1., 
            duration=1000, loopCount=-1, 
            valueChanged=self.updateGradient
        )

        self.focusWidget = None
        self.progress = None
        self.hide()
        parent.installEventFilter(self)

    def start(self):
        self.show()
        self.raise_()
        self.focusWidget = QtWidgets.QApplication.focusWidget()
        self.setFocus()
        self.animation.start()

    def stop(self):
        self.hide()
        self.animation.stop()
        if self.focusWidget:
            self.focusWidget.setFocus()
            self.focusWidget = None

    def updateGradient(self, value):
        self.gradient.setAngle(-value * 360)
        self.update()

    def setValue(self, value):
        self.progress = value
        self.update()

    def eventFilter(self, source, event):
        if event.type() == QtCore.QEvent.Resize:
            self.setGeometry(source.rect())
        return super().eventFilter(source, event)

    def showEvent(self, event):
        self.setGeometry(self.parent().rect())
        self.animation.start()

    def hideEvent(self, event):
        self.animation.stop()

    def paintEvent(self, event):
        qp = QtGui.QPainter(self)
        qp.setRenderHints(qp.Antialiasing)
        color = self.palette().window().color()
        color.setAlpha(max(color.alpha() * .5, 128))
        qp.fillRect(self.rect(), color)

        text = self.text if self.progress is None else f'{self.text} {self.progress}%'
        textWidth = self.fontMetrics().horizontalAdvance(text)
        textHeight = self.fontMetrics().height()

        if textWidth > self.width() or textHeight * 3 > self.height():
            drawText = False
            size = max(0, min(self.width(), self.height()) - textHeight * 2)
        else:
            size = min(self.height() / 3, max(textWidth, textHeight))
            drawText = True

        circleRect = QtCore.QRect(0, 0, size, size)
        circleRect.moveCenter(self.rect().center())

        if drawText:
            circleRect.moveTop(circleRect.top() - textHeight)
            middle = circleRect.center().x()
            qp.drawText(
                int(middle - textWidth / 2), int(circleRect.bottom() + textHeight), 
                int(textWidth), int(textHeight), 
                QtCore.Qt.AlignCenter, text)

        self.gradient.setColorAt(.5, self.palette().windowText().color())
        qp.setPen(QtGui.QPen(self.gradient, textHeight))
        qp.drawEllipse(circleRect)

class LoadingExtension:
    def __init__(self, parent):
        self.parent = parent
        self.loader = None

    def startLoading(self, text=None):
        if self.loader is None:
            self.loader = Loader(text if text else 'Loading...', self.parent)
            self.loader.setWindowModality(QtCore.Qt.WindowModal)
        self.loader.start()

    def setProgress(self, value):
        self.loader.setValue(value)

    def loadingFinished(self):
        if self.loader:
            self.loader.stop()