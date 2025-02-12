############### Plotter ################
import numpy as np
import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtGui, QtWidgets
import time

class Plotter():
    def __init__(self, plot_title=None, plot_grid=True, plot_legend=True, history_length=300, plot_size=(640, 360), as_app=False):
        self.num_plots = 0
        self.plot_names = []
        self.plot_colors = [pg.intColor(i) for i in range(10000)]
        self.plot_labels = None
        self.plot_yrange = None
        self.plot_xrange = None
        self.plot_title = None
        self.plot_grid = plot_grid
        self.plot_legend = plot_legend
        self.plot_size = plot_size
        self.history_length = history_length


        if as_app:
            self.app = QtWidgets.QApplication([])
        self.win = pg.GraphicsLayoutWidget(title=plot_title)
        pg.setConfigOption('background', (255,255,255, 100))
        # self.win.setBackground(None)
        self.win.resize(*plot_size)
        self.plots = []
        self.curves = []
        self.data = []
        self.t0 = time.time()
        self.t = []

        # display the window
        if as_app:
            self.win.show()
            # self.win.setWindowFlags(QtCore.Qt.WindowType.WindowStaysOnTopHint)
        # self.win.show()

    def add_plot(self, name, labels, legends):
        self.plot_names.append(name)
        self.plots.append(self.win.addPlot(title=name))
        self.plots[-1].setLabel('left', text=labels[0])
        self.plots[-1].setLabel('bottom', text=labels[1])
        self.plots[-1].showGrid(x=self.plot_grid, y=self.plot_grid)
        self.plots[-1].enableAutoRange('xy', True)
        # one curve for each legend
        if self.plot_legend:
            self.plots[-1].addLegend()

        self.curves.append([])
        for i in range(len(legends)):
            self.curves[-1].append(self.plots[-1].plot(pen=self.plot_colors[i] if self.plot_colors is not None else None, name=legends[i]))
        self.data.append(np.array([]))
        self.t.append(np.array([]))
        self.num_plots += 1
        if self.num_plots % 3 == 0:
            self.win.nextRow()
    

    def plot(self, name, value, labels, legends):
        if name not in self.plot_names:
            self.add_plot(name, labels, legends)
        i = self.plot_names.index(name)
        timestamp = np.array([time.time() - self.t0])
        value = np.array([value])
        if len(self.data[i]) == 0:
            self.data[i] = value
            self.t[i] = timestamp
        else:
            self.data[i] = np.concatenate((self.data[i], value))
            self.t[i] = np.concatenate((self.t[i], timestamp))
        #self.curves[i].setData(self.t[i][-self.history_length:], self.data[i][-self.history_length:])
        for j in range(len(self.curves[i])):
            self.curves[i][j].setData(self.t[i][-self.history_length:], self.data[i][-self.history_length:,j])

    def clear(self):
        for i in range(self.num_plots):
            self.data[i] = np.array([])
            self.t[i] = np.array([])
            for j in range(len(self.curves[i])):
                self.curves[i][j].clear()
            self.t0 = time.time()
    
    def update(self):
        if self.app is not None:
            self.app.processEvents()

    def set_yrange(self, yrange):
        for i in range(self.num_plots):
            self.plots[i].enableAutoRange('xy', False)
            self.plots[i].setLimits(yMin=yrange[0], yMax=yrange[1])


'''
# a simple example of how to use the Plotter class, just plot a multivariate sine wave and in another plot, plot the magnitude of the sine wave
if __name__ == '__main__':
    plotter = Plotter(plot_title='Plotter Example', plot_grid=True, plot_legend=True, plot_size=(800, 600))
    t = 0
    while True:
        plotter.plot('sine wave', np.array([np.sin(t), np.cos(t)]), labels=('y', 's'), legends=('sin', 'cos'))
        plotter.plot('magnitude', np.array([np.abs(np.sin(t))]), labels=('y', 's'), legends=('magnitude',))
        plotter.update()
        t += .01
        time.sleep(.01)
'''