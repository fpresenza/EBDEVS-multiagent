import numpy as np
from matplotlib.collections import LineCollection
from matplotlib.animation import FuncAnimation
import collections


class Animate(object):
    def __init__(self, fig, ax, timestep, frames, maxlen=1):
        self.fig = fig
        self.ax = ax
        self.h = timestep
        self.frames = frames
        self.x_tail = collections.deque(maxlen=maxlen)
        Xk = self.frames[0][1]
        self.x_tail.append(Xk)
        self.stamp = self.ax.text(
            0.01, 0.01, r'{:.2f} secs'.format(0.0),
            verticalalignment='bottom', horizontalalignment='left',
            transform=self.ax.transAxes, color='green', fontsize=10)
        self.anim = None
        self.teams = {}
        edgestyle = {'color': '0.2', 'linewidth': 0.7, 'zorder': 10}
        self.set_edgestyle(**edgestyle)
        self._extra_artists = None

    def set_teams(self, teams):
        self.teams = teams.copy()
        for name, data in teams.items():
            style = data.get(
                'style', {'color': 'b', 'marker': 'o', 'markersize': '5'}
            )
            style.update(ls='', label=name)
            line = self.ax.plot([], [], **style)
            self.teams[name]['points'] = line[0]
            if data.get('tail') is True:
                style.update(markersize=0.5, alpha=0.4)
                style.pop('label')
                line = self.ax.plot([], [], **style)
                self.teams[name]['tail'] = line[0]
            else:
                self.teams[name]['tail'] = None

    def set_edgestyle(self, **style):
        self.edges = LineCollection([], **style)
        self.ax.add_artist(self.edges)

    def _update_extra_artists(self, frame):
        pass

    def set_extra_artists(self, *artists):
        self._extra_artists = []
        for artist in artists:
            self._extra_artists.append(artist)

    def update(self, frame):
        tk, Xk, Ek, Tk = frame[:4]
        q = np.array(self.x_tail)
        self.x_tail.append(Xk)
        for data in self.teams.values():
            inteam = Tk == data['id']
            points = data['points']
            tail = data.get('tail')

            x = Xk[inteam]
            points.set_data(x[:, 0], x[:, 1])
            if tail is not None:
                tail.set_data(q[:, inteam, 0], q[:, inteam, 1])
        self.stamp.set_text('$t = {:.3f} s$'.format(tk))
        if len(Ek) > 0:
            self.edges.set_segments(Xk[Ek])
        else:
            self.edges.set_segments([])
        self._update_extra_artists(frame)
        return self.ax.lines + self.ax.artists + self.ax.texts

    def run(self, file=None):
        self.anim = FuncAnimation(
            self.fig,
            self.update,
            frames=self.frames,
            interval=1000 * self.h,
            blit=True
        )
        if file:
            self.anim.save(
                file,
                fps=1. / self.h,
                dpi=200,
                extra_args=['-vcodec', 'libx264'])
