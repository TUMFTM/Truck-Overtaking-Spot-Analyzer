import multiprocessing
import queue
import tkinter as tk
import warnings
from multiprocessing import freeze_support
from tkinter import Frame, Scale, IntVar, Checkbutton, StringVar, OptionMenu, Label, Radiobutton

import matplotlib
import matplotlib.patches as patches
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk

from OvertakingSpotAnalyzer import OsaTrigger

matplotlib.use("TkAgg")

LARGE_FONT = ("Verdana", 20)
tum_color = {1: (0 / 255, 101 / 255, 189 / 255), 2: (227 / 255, 114 / 255, 34 / 255), 3: (0, 82 / 255, 147 / 255),
             4: (162 / 255, 173 / 255, 0 / 255)}


class OvertakingSpotAnalyzerGUI(tk.Tk):

    def __init__(self, queue_to_osa, queue_to_gui, *args, **kwargs):
        tk.Tk.__init__(self, *args, **kwargs)
        tk.Tk.wm_title(self, "Overtaking Spot Analyzer")

        self.queue_to_osa = queue_to_osa
        self.queue_to_gui = queue_to_gui
        self.started = False

        sidebar_width = 5
        Label(self, text="Overtaking Spot Analyzer", font=LARGE_FONT).grid(row=0, column=0, columnspan=sidebar_width)
        self.hill = self.generate_hill_gui(sidebar_width)
        self.truck_a = self.generate_truck_gui('A', sidebar_width)
        self.truck_b = self.generate_truck_gui('B', sidebar_width)
        self.calculation = self.generate_calculation_gui(sidebar_width)
        self.coop_selection = self.generate_coop_selection_gui(sidebar_width)
        self.cost_parameter = self.generate_cost_parameter_gui(sidebar_width)
        self.canvas, self.plots = self.generate_canvas_and_plots(sidebar_width)

        self.set_start_config()

        self.started = True
        self.osa_result = None
        self.generate_slope()

    def generate_slope(self, *args, **kwargs):
        """Generated the road slope from the user input"""
        if self.started:

            if len(args) > 0 and args[0] != 'profile':
                self.hill['profile'].set('0 Custom')

            degree = int(self.hill['deg'].get())
            length = int(self.hill['len'].get())

            x = [length * 0.25 * i for i in range(5)]
            y = [s.get() for s in self.hill['y']]

            with warnings.catch_warnings():
                warnings.simplefilter("ignore")
                z = np.polyfit(x, y, degree)

            polynomial_function = np.poly1d(z)
            xs = list(range(0, length + 50, 50))
            ys = [polynomial_function(e) for e in xs]

            slope = [(ys[i + 1] - ys[i]) * 10 for i in range(len(ys) - 1)]

            self.hill['slope'] = slope

            self.plot_slope(x, y, xs, ys, slope)
            if min(slope) >= -20 and max(slope) <= 20:
                self.trigger_osa_query()

    def trigger_osa_query(self, *args, **kwargs):
        """Triggers the OSA by pushing the current parameters specified by the user into the pipeline to osa"""
        if self.started:
            truck_a_values = {k: v.get() for (k, v) in self.truck_a.items()}
            truck_b_values = {k: v.get() for (k, v) in self.truck_b.items()}
            step_size = int(self.calculation['step'].get())
            cost_param = {'type': self.cost_parameter['param'].get(), 'v_car': self.cost_parameter['v_car'].get() / 3.6}
            self.queue_to_osa.put(['query', truck_a_values, truck_b_values, self.hill['slope'], step_size, cost_param])

    def check_for_osa_response(self):
        """Periodically checks the pipeline for an response from the osa and if any plots the answer"""
        while True:
            try:
                data_from_osa = self.queue_to_gui.get_nowait()

                if data_from_osa[0] == 'response':
                    self.osa_result = data_from_osa[1]

                    self.plot_velocity_profile()
                    self.plot_overtaking_cooperation('pc')
                    self.plot_overtaking_cooperation('ac')
                    self.plot_costs()
                    self.show_calculation_time()
                    self.draw_plots()

            except queue.Empty:
                break

        self.after(50, self.check_for_osa_response)

    ##########################################################
    #                   Display Results                      #
    ##########################################################

    def plot_slope(self, x, y, xs, ys, slope):
        """Plots the road height profile as well as the slope"""
        self.plots['height'].clear()
        self.plots['slope'].clear()

        self.plots['height'].scatter(x, y)
        lns1 = self.plots['height'].plot(xs, ys, color=tum_color[1], label='Road Height')
        lns2 = self.plots['slope'].plot(xs[:-1], slope, color=tum_color[2], label='Road Slope')
        self.plots['slope'].axhline(0, color='black', linewidth=0.3)

        self.plots['slope'].set(ylim=(-8, 8))
        self.plots['slope'].set_ylabel('Road Slope [%]')
        self.plots['height'].set_ylabel('Road Height [m]')
        self.plots['height'].set(xlim=(0, int(self.hill['len'].get())), ylim=(min([0] + ys) - 1, max([10] + ys) + 1))

        # Combine Legends
        lns = lns1 + lns2
        labs = [ln.get_label() for ln in lns]
        self.plots['height'].legend(lns, labs, loc=3)
        if min(slope) < -20 or max(slope) > 20:
            self.plots['slope'].text(x[-1] / 2, -0.66, 'Slope is too extreme', fontsize=18, ha='center', wrap=True)
            for e in ['none', 'first', 'stvo', 'pc', 'ac', 'vel', 'time_pc', 'fuel_pc', 'time_ac', 'fuel_ac']:
                self.plots[e].clear()
            self.draw_plots()

    def plot_velocity_profile(self):
        """Plots the velocity profile received by the OSA and selected by the user"""
        truck_a = self.osa_result['truck_a']
        truck_b = self.osa_result['truck_b']
        first_contact = self.osa_result['first_contact']
        overtaking = self.osa_result['overtaking']

        start_a_index = truck_a['start'] // 10
        start_b_index = truck_b['start'] // 10

        xs = list(range(0, int(len(truck_a['v_profile']) * 10), 10))

        a_xs = xs[start_a_index:]
        b_xs = xs[start_b_index:]
        a_m_vel = [e * 3.6 for e in truck_a['v_profile_max'][start_a_index:]]
        b_m_vel = [e * 3.6 for e in truck_b['v_profile_min'][start_b_index:]]

        coop_mode = self.coop_selection.get()
        if coop_mode == 'Solo' or overtaking[coop_mode.lower()] is None or overtaking[coop_mode.lower()]['x'] is None:
            self.coop_selection.set('Solo')
            coop_mode = 'Solo'
            a_vel = [e * 3.6 for e in truck_a['v_profile'][start_a_index:]]
            b_vel = [e * 3.6 for e in truck_b['v_profile'][start_b_index:]]
            start_overtaking = None
        else:
            a_vel = [e * 3.6 for e in overtaking[coop_mode.lower()]['a_profile']]
            b_vel = [e * 3.6 for e in overtaking[coop_mode.lower()]['b_profile']]
            start_overtaking = overtaking[coop_mode.lower()]['x']

        self.plots['vel'].clear()
        self.plots['vel'].plot(a_xs, a_vel, label='Truck A - Mode: ' + coop_mode, color=tum_color[1])
        self.plots['vel'].plot(a_xs, a_m_vel, linestyle='--', color=tum_color[1])
        self.plots['vel'].plot(b_xs, b_vel, label='Truck B - Mode: ' + coop_mode, color=tum_color[2])
        self.plots['vel'].plot(b_xs, b_m_vel, linestyle='--', color=tum_color[2])

        if first_contact['contact']:
            self.plots['vel'].axvline(first_contact['index_a'] * 10, color=tum_color[1], linewidth=0.3)
            if start_overtaking:
                self.plots['vel'].axvline(start_overtaking, color=tum_color[4], linewidth=0.3)

        self.plots['vel'].set(ylim=(min(59, min(a_vel + b_m_vel)), max(101, max(b_vel + a_m_vel))))
        self.plots['vel'].set_ylabel('Velocity Profile [km/h]')
        self.plots['vel'].legend(loc=3)

    def plot_overtaking_cooperation(self, coop):
        """Plots the details for PC and AC overtaking on evey possible staring position"""
        if coop == 'ac':
            time_plot = self.plots['time_ac']
            fuel_plot = self.plots['fuel_ac']
            title = 'Active Cooperation'
            details = self.osa_result['ac_details']
        else:
            time_plot = self.plots['time_pc']
            fuel_plot = self.plots['fuel_pc']
            title = 'Passive Cooperation'
            details = self.osa_result['pc_details']

        hill_l = int(self.hill['len'].get())
        xs = details['xs']
        a_times = details['a_times']
        b_times = details['b_times']
        a_fuels = details['a_fuels']
        b_fuels = details['b_fuels']
        overtaking_times = details['durations']

        time_plot.clear()
        fuel_plot.clear()
        time_plot.set_ylabel('Time [s]')
        fuel_plot.set_ylabel('Fuel [l]')
        if coop == 'ac':
            time_plot.set_xlabel('Longitudinal Position on Road [m]')

        if len(xs) > 0:
            a_t_diffs = [e - self.osa_result['overtaking']['first']['a_time'] for e in a_times]
            a_f_diffs = [e - self.osa_result['overtaking']['first']['a_fuel'] for e in a_fuels]
            b_t_diffs = [e - self.osa_result['overtaking']['first']['b_time'] for e in b_times]
            b_f_diffs = [e - self.osa_result['overtaking']['first']['b_fuel'] for e in b_fuels]

            max_y = max(a_t_diffs + overtaking_times + [60] + b_t_diffs)
            min_y = min(a_t_diffs + b_t_diffs + [0])
            rect_pre = patches.Rectangle((-5, min_y - 5), xs[0], max_y + 15 - min_y,
                                         edgecolor='none', facecolor='whitesmoke')
            time_plot.add_patch(rect_pre)
            rect_post = patches.Rectangle((xs[-1], min_y - 5), hill_l - xs[-1] + 5, max_y + 15 - min_y,
                                          edgecolor='none', facecolor='whitesmoke')
            time_plot.add_patch(rect_post)

            lns0 = time_plot.plot(xs, overtaking_times, label='Overtaking Time', color='black')
            lns1 = time_plot.plot(xs, a_t_diffs, label='Time Diff Truck A to Solo', color=tum_color[1])
            lns2 = fuel_plot.plot(xs, a_f_diffs, label='Fuel Diff Truck A to Solo', color=tum_color[1], linestyle='--')

            lns3 = time_plot.plot(xs, b_t_diffs, label='Time Diff Truck B to Solo', color=tum_color[2])
            lns4 = fuel_plot.plot(xs, b_f_diffs, label='Fuel Diff Truck B to Solo', color=tum_color[2], linestyle='--')

            time_plot.axhline(45, color='black', linewidth=0.3)

            lns = lns0 + lns1 + lns2 + lns3 + lns4
            labs = [ln.get_label() for ln in lns]
            time_plot.legend(lns, labs, loc=3)

            time_plot.set(ylim=(min_y - 1, max_y + 5))
            time_plot.text(30, (max_y + 5 - min_y) * 0.91 + min_y, title, fontsize=12, ha='left')
            time_plot.text(30, (max_y + 5 - min_y) * 0.78 + min_y,
                           "Details to every Starting Point \nGray: No overtaking possible", fontsize=8, ha='left')


        else:
            rect_full = patches.Rectangle((-5, -5), hill_l + 5, 75, edgecolor='none', facecolor='whitesmoke')
            time_plot.add_patch(rect_full)
            time_plot.set(ylim=(-1, 65))
            time_plot.text(30, 65 * 0.91, title, fontsize=12, ha='left')

            time_plot.text(hill_l / 2, 30, 'No overtaking possible', fontsize=18, ha='center', wrap=True)

    def plot_costs(self):
        """Plots the cost overview for each Overtaking scenario"""
        modes = ['None', 'First', 'StVO', 'PC', 'AC']
        for mode in modes:
            self.plots[mode.lower()].clear()
            if self.osa_result['overtaking'][mode.lower()] and self.osa_result['overtaking'][mode.lower()]['x']:

                width = 0.35
                a_t = self.osa_result['overtaking'][mode.lower()]['cost']['c_a_time']
                a_f = self.osa_result['overtaking'][mode.lower()]['cost']['c_a_fuel']
                b_t = self.osa_result['overtaking'][mode.lower()]['cost']['c_b_time']
                b_f = self.osa_result['overtaking'][mode.lower()]['cost']['c_b_fuel']
                c_t = self.osa_result['overtaking'][mode.lower()]['cost']['c_car']
                total = sum(self.osa_result['overtaking'][mode.lower()]['cost'].values())

                if self.cost_parameter['display'].get() == 'Rel':
                    a_t_ref = self.osa_result['overtaking']['ac']['cost']['c_a_time']
                    a_f_ref = self.osa_result['overtaking']['ac']['cost']['c_a_fuel']
                    b_t_ref = self.osa_result['overtaking']['ac']['cost']['c_b_time']
                    b_f_ref = self.osa_result['overtaking']['ac']['cost']['c_b_fuel']
                    c_t_ref = self.osa_result['overtaking']['ac']['cost']['c_car']
                    total_ref = sum(self.osa_result['overtaking']['ac']['cost'].values())

                    a_t = a_t - a_t_ref
                    a_f = a_f - a_f_ref
                    b_t = b_t - b_t_ref
                    b_f = b_f - b_f_ref
                    c_t = c_t - c_t_ref
                    total = total - total_ref

                rect1 = self.plots[mode.lower()].bar(1 - width / 2, a_t, width=width,
                                                     color=(51 / 255, 160 / 255, 255 / 255),
                                                     ls='-', linewidth=2, edgecolor=tum_color[1])
                rect2 = self.plots[mode.lower()].bar(1 + width / 2, a_f, width=width,
                                                     color=(51 / 255, 160 / 255, 255 / 255),
                                                     ls='--', linewidth=2, edgecolor=tum_color[1])

                rect3 = self.plots[mode.lower()].bar(2 - width / 2, b_t, width=width,
                                                     color=(234 / 255, 151 / 255, 93 / 255),
                                                     ls='-', linewidth=2, edgecolor=tum_color[2])
                rect4 = self.plots[mode.lower()].bar(2 + width / 2, b_f, width=width,
                                                     color=(234 / 255, 151 / 255, 93 / 255),
                                                     ls='--', linewidth=2, edgecolor=tum_color[2])

                rect5 = self.plots[mode.lower()].bar(3 - width / 2, c_t, width=width, color=tum_color[3],
                                                     ls='-', linewidth=2, edgecolor=tum_color[3])

                rect6 = self.plots[mode.lower()].bar(3 + width * 1.5, total, width=width, color=tum_color[4],
                                                     ls='-', linewidth=2, edgecolor=tum_color[4])

                self.plots[mode.lower()].set_xticks([1, 2, 3 - width / 2, 3 + width * 1.5])
                self.plots[mode.lower()].set_xticklabels(['Truck A', 'Truck B', 'Car', 'Total'])
                self.plots[mode.lower()].axhline(y=0, linewidth=.5, color='k')

                def auto_label(rects):
                    """Attach a text label above each bar in *rects*, displaying its height."""
                    for rect in rects:
                        height = round(rect.get_height(), 2)
                        if height >= 0:
                            offset = 2
                        else:
                            offset = -10

                        self.plots[mode.lower()].annotate('{}'.format(height),
                                                          xy=(rect.get_x() + rect.get_width() / 2, height),
                                                          xytext=(0, offset),
                                                          textcoords="offset points", size=6,
                                                          ha='center', va='bottom')

                auto_label(rect1)
                auto_label(rect2)
                auto_label(rect3)
                auto_label(rect4)
                auto_label(rect5)
                auto_label(rect6)

                if mode == 'None':
                    self.plots[mode.lower()].set_title('{}'.format(mode))
                    if self.cost_parameter['display'].get() == 'Rel':
                        self.plots[mode.lower()].set_ylabel('Relative Costs [€]')
                    else:
                        self.plots[mode.lower()].set_ylabel('Costs [€]')
                else:
                    duration = self.osa_result['overtaking'][mode.lower()]['duration']
                    if duration:
                        if self.cost_parameter['display'].get() == 'Rel':
                            duration = duration - self.osa_result['overtaking']['ac']['duration']
                        self.plots[mode.lower()].set_title('{}: {}s'.format(mode, round(duration)))
                    else:
                        self.plots[mode.lower()].clear()
                        self.plots[mode.lower()].set_title('{}: {}'.format(mode, 'Not possible'))
                        self.plots[mode.lower()].set_xticks([])

                if mode == 'AC':
                    if self.cost_parameter['display'].get() == 'Rel':
                        self.plots['ac'].clear()
                        self.plots['ac'].set_title('{}: {}'.format('AC', 'Reference'))
                        self.plots['ac'].set_xticks([])
                        self.plots[mode.lower()].set_ylim(self.plots[mode.lower()].axes.get_ylim()[0] * 1.08,
                                                          self.plots[mode.lower()].axes.get_ylim()[1] * 1.08)
                    else:
                        self.plots[mode.lower()].set_ylim(self.plots[mode.lower()].axes.get_ylim()[0] * 1.03,
                                                          self.plots[mode.lower()].axes.get_ylim()[1] * 1.03)

            else:
                self.plots[mode.lower()].clear()
                self.plots[mode.lower()].set_title('{}: {}'.format(mode, 'Not possible'))
                self.plots[mode.lower()].set_xticks([])

    def show_calculation_time(self):
        """Displays the time required for the calculation by the osa"""
        self.calculation['time_label'].config(text='Time: ' + str(round(self.osa_result['calculation_time'], 2)) + 's')

    def draw_plots(self):
        """Triggers matplotlib to draw the plots, can be used to export plots to files"""
        self.canvas[0].draw()
        self.canvas[1].draw()
        # plt.savefig('costs.png', dpi=500, bbox_inches='tight')

    ##########################################################
    #           Generate and Manage GUI Elements             #
    ##########################################################

    def set_start_config(self):
        """Sets the start configuration that can be then altered by the user"""
        self.truck_a['vel'].set(85)
        self.truck_a['weight'].set(25)
        self.truck_a['vel_p'].set(5)
        self.truck_a['vel_m'].set(-5)

        self.truck_b['start'].set(200)
        self.truck_b['vel'].set(82)
        self.truck_b['weight'].set(35)
        self.truck_b['vel_p'].set(5)
        self.truck_b['vel_m'].set(-5)

        self.calculation['step'].set(8)

        self.hill['len'].set(7000)

    def generate_canvas_and_plots(self, width):
        """Generated the Canvas for the Plots on the right side of the GUI"""
        fig, ax = plt.subplots(4, 1, sharex='all', squeeze=False)
        fig.set_size_inches(12, 8.5)  # 12, 9 for displays with 1400px vertical
        fig.tight_layout()
        fig.subplots_adjust(top=0.98, wspace=0.25, hspace=0.05, left=0.06, right=0.94, bottom=0.06)

        plots = {'height': ax[0][0], 'slope': ax[0][0].twinx(),
                 'vel': ax[1][0],
                 'time_pc': ax[2][0], 'fuel_pc': ax[2][0].twinx(),
                 'time_ac': ax[3][0], 'fuel_ac': ax[3][0].twinx()}

        canvas_lines = FigureCanvasTkAgg(fig, self)
        canvas_lines.get_tk_widget().grid(row=0, column=width, columnspan=1, rowspan=14)

        toolbar_frame = Frame(master=self)
        toolbar_frame.grid(row=17, column=width)
        NavigationToolbar2Tk(canvas_lines, toolbar_frame)

        fig, ax = plt.subplots(1, 5, sharey='all', squeeze=False)
        fig.set_size_inches(12, 2.3)  # 2.5
        fig.tight_layout()
        fig.subplots_adjust(top=0.90, wspace=0.05, hspace=0.05, left=0.06, right=0.94, bottom=0.1)
        canvas_bars = FigureCanvasTkAgg(fig, self)
        canvas_bars.get_tk_widget().grid(row=14, column=width, columnspan=1, rowspan=3)

        for (i, e) in enumerate(['none', 'first', 'stvo', 'pc', 'ac']):
            plots[e] = ax[0][i]

        return [canvas_lines, canvas_bars], plots

    def generate_truck_gui(self, name, width):
        """Generates all GUI elements required for parametrisation of the trucks"""
        if name == 'A':
            row = 4
        else:
            row = 7

        truck = {'start': IntVar(), 'vel': IntVar(), 'weight': IntVar(), 'vel_p': IntVar(), 'vel_m': IntVar()}

        Label(self, text="Parameter Setting for Truck " + name).grid(row=row, column=0, columnspan=width)

        Scale(self, from_=3000, to=0, label="Start", command=self.check_a_start_pos,
              variable=truck['start'], resolution=10).grid(row=row + 1, column=0, rowspan=2)

        Scale(self, from_=40, to=10, label="Weight", command=self.trigger_osa_query,
              variable=truck['weight']).grid(row=row + 1, column=1, rowspan=2)

        Scale(self, from_=90, to=60, label="Vel", command=self.trigger_osa_query,
              variable=truck['vel']).grid(row=row + 1, column=2, rowspan=2)

        Scale(self, from_=10, to=0, label="Vel+", command=self.trigger_osa_query,
              length=50, variable=truck['vel_p']).grid(row=row + 1, column=3, rowspan=1)

        Scale(self, from_=0, to=-10, label="Vel-", command=self.trigger_osa_query,
              length=50, variable=truck['vel_m']).grid(row=row + 2, column=3, rowspan=1)

        Label(self, text="Start [m]\nWeight [t]\n Vel [km/h]").grid(row=row, column=4, rowspan=2)

        return truck

    def generate_hill_gui(self, width):
        """Generates all GUI elements required for parametrisation of the road height/slope """
        Label(self, text="Road height and length in m and polynomial degree").grid(row=1, column=0, columnspan=width)

        hill = {'y': [], 'len': IntVar(), 'deg': IntVar(), 'profile': StringVar()}

        for i in range(5):
            hill['y'].append(IntVar())
            text = str(25 * i) + '%'
            Scale(self, from_=100, to=0, label=text, command=self.generate_slope,
                  variable=hill['y'][i]).grid(row=2, column=i)

        Scale(self, from_=5000, to=10000, resolution=100, label="Length", orient='horizontal',
              variable=hill['len'], command=self.generate_slope).grid(row=3, column=0, columnspan=2)

        Scale(self, from_=4, to=10, resolution=1, label="Degree", orient='horizontal',
              variable=hill['deg'], command=self.generate_slope).grid(row=3, column=2, columnspan=1)

        hill_profiles = ["0 Custom",
                         "1 Down-Up", "2 Flat-Up", "3 Up-Up",
                         "4 Up-Flat", "5 Flat-Flat", "6 Down-Flat",
                         "7 Down-Down", "8 Flat-Down", "9 Up-Down"]
        hill['profile'].set(hill_profiles[0])
        OptionMenu(self, hill['profile'], *hill_profiles, command=self.set_hill_profile).grid(row=3, column=3,
                                                                                              columnspan=2)

        return hill

    def generate_calculation_gui(self, width):
        """Generates all GUI elements required for setting the Stepsize, live Update and displaying the time"""
        Label(self, text="Calculation Step Size in meter and Required Time").grid(row=10, column=0, columnspan=width)

        calculation = {'step': IntVar(), 'update': IntVar()}

        Scale(self, from_=2, to=24, resolution=2, label="Step Size", orient='horizontal', variable=calculation['step'],
              command=self.trigger_osa_query).grid(row=11, column=0, columnspan=2)

        Checkbutton(self, text="Update", variable=calculation['update'],
                    command=self.switch_update).grid(row=11, column=2)
        calculation['update'].set(1)

        time_label = Label(self, text="Time -")
        time_label.grid(row=11, column=3, columnspan=2)
        calculation['time_label'] = time_label

        return calculation

    def generate_coop_selection_gui(self, width):
        """Generates the GUI elements to select which velocity profile should be visualized"""
        Label(self, text="Which velocity profile should be visualized?").grid(row=12, column=0, columnspan=width - 1)

        coop_selection = StringVar()
        modes = ['Solo', 'None', 'First', 'Stvo', 'PC', 'AC']
        coop_selection.set(modes[0])

        Radiobutton(self, text=modes[0], variable=coop_selection, value=modes[0],
                    command=self.changed_visualized_coop_mode).grid(row=12, column=width - 1)

        for i, mode in enumerate(modes[1:]):
            Radiobutton(self, text=mode, variable=coop_selection, value=mode,
                        command=self.changed_visualized_coop_mode).grid(row=13, column=i)

        return coop_selection

    def generate_cost_parameter_gui(self, width):
        """Generates all GUI elements required to select the parameters for costs calculation"""
        Label(self, text="Parameter and display setting for cost calculation").grid(row=14, column=0, columnspan=width)

        cost_parameter = {'param': StringVar(), 'v_car': IntVar(), 'display': StringVar()}

        modes = ['Bourdon', 'Kock']
        cost_parameter['param'].set(modes[0])
        for i, mode in enumerate(modes):
            Radiobutton(self, text=mode, variable=cost_parameter['param'], value=mode,
                        command=self.trigger_osa_query).grid(row=15, column=i)

        Scale(self, from_=80, to=160, resolution=10, label="Car vel [km/h]", orient='horizontal',
              variable=cost_parameter['v_car'],
              command=self.trigger_osa_query).grid(row=15, column=2, columnspan=1)
        cost_parameter['v_car'].set(120)

        modes = ['Abs', 'Rel']
        cost_parameter['display'].set(modes[0])
        for i, mode in enumerate(modes, start=3):
            Radiobutton(self, text=mode, variable=cost_parameter['display'], value=mode,
                        command=self.changed_visualized_cost_mode).grid(row=15, column=i)


        #Label(self, text="______________________________________________________________________________________________").grid(row=16, column=0, columnspan=width)
        t = Label(self, text="""
Cooperative Truck Overtaking on Freeways
Fifteenth International Conference on Ecological Vehicles and Renewable Energies
Mertens, Jan Cedric; Jahn, Lennard; Hauenstein, Jürgen; Diermeyer, Frank; Kraus, Sven
2020""", anchor="w",justify="left")
        t.config(font=("Verdana", 7))
        t.grid(row=16, column=0, columnspan=width)
       # Label(self, text="______________________________________________________________________________________________\n\n\n").grid(row=18, column=0, columnspan=width)


        return cost_parameter

    def set_hill_profile(self, *args, **kwargs):
        """Sets the pre-defined hill profiles in the drop-down menu"""
        f = 7
        if self.hill['profile'].get().startswith("1"):
            self.hill['y'][0].set(2 * f)
            self.hill['y'][1].set(1 * f)
            self.hill['y'][2].set(0 * f)
            self.hill['y'][3].set(1 * f)
            self.hill['y'][4].set(2 * f)
        elif self.hill['profile'].get().startswith("2"):
            self.hill['y'][0].set(0 * f)
            self.hill['y'][1].set(0 * f)
            self.hill['y'][2].set(0 * f)
            self.hill['y'][3].set(1 * f)
            self.hill['y'][4].set(2 * f)
        elif self.hill['profile'].get().startswith("3"):
            self.hill['y'][0].set(0 * f)
            self.hill['y'][1].set(1 * f)
            self.hill['y'][2].set(2 * f)
            self.hill['y'][3].set(3 * f)
            self.hill['y'][4].set(4 * f)
        elif self.hill['profile'].get().startswith("4"):
            self.hill['y'][0].set(0 * f)
            self.hill['y'][1].set(1 * f)
            self.hill['y'][2].set(2 * f)
            self.hill['y'][3].set(2 * f)
            self.hill['y'][4].set(2 * f)
        elif self.hill['profile'].get().startswith("5"):
            self.hill['y'][0].set(0 * f)
            self.hill['y'][1].set(0 * f)
            self.hill['y'][2].set(0 * f)
            self.hill['y'][3].set(0 * f)
            self.hill['y'][4].set(0 * f)
        elif self.hill['profile'].get().startswith("6"):
            self.hill['y'][0].set(2 * f)
            self.hill['y'][1].set(1 * f)
            self.hill['y'][2].set(0 * f)
            self.hill['y'][3].set(0 * f)
            self.hill['y'][4].set(0 * f)
        elif self.hill['profile'].get().startswith("7"):
            self.hill['y'][0].set(4 * f)
            self.hill['y'][1].set(3 * f)
            self.hill['y'][2].set(2 * f)
            self.hill['y'][3].set(1 * f)
            self.hill['y'][4].set(0 * f)
        elif self.hill['profile'].get().startswith("8"):
            self.hill['y'][0].set(2 * f)
            self.hill['y'][1].set(2 * f)
            self.hill['y'][2].set(2 * f)
            self.hill['y'][3].set(1 * f)
            self.hill['y'][4].set(0 * f)
        elif self.hill['profile'].get().startswith("9"):
            self.hill['y'][0].set(0 * f)
            self.hill['y'][1].set(1 * f)
            self.hill['y'][2].set(2 * f)
            self.hill['y'][3].set(1 * f)
            self.hill['y'][4].set(0 * f)

        self.generate_slope('profile')

    def check_a_start_pos(self, *args, **kwargs):
        """Check that Truck A starts behind Truck B"""
        if self.truck_b['start'].get() - self.truck_a['start'].get() < 100:
            self.truck_a['start'].set(int(self.truck_b['start'].get() - 100))
        else:
            self.trigger_osa_query()

    def check_b_start_pos(self, *args, **kwargs):
        """Check that Truck A starts behind Truck B"""
        if self.truck_b['start'].get() - self.truck_a['start'].get() < 100:
            self.truck_b['start'].set(int(self.truck_a['start'].get() + 100))
        else:
            self.trigger_osa_query()

    def switch_update(self, *args, **kwargs):
        """Switch live Update on/off"""
        if self.started:
            self.started = False
        else:
            self.started = True
            self.generate_slope()

    def changed_visualized_coop_mode(self):
        """Trigger a new drawing of the velocity profiles"""
        self.plot_velocity_profile()
        self.draw_plots()

    def changed_visualized_cost_mode(self):
        """Trigger a new drawing of the costs"""
        self.plot_costs()
        self.draw_plots()


if __name__ == '__main__':
    freeze_support()

    # Two Queues (pipelines) are used for the communication between OSA and GUi process
    q_gui_to_osa = multiprocessing.Queue()
    q_osa_to_gui = multiprocessing.Queue()

    # Starts the OSA in a separated process
    osa_process = multiprocessing.Process(None, OsaTrigger.wait_for_gui_input, args=(q_gui_to_osa, q_osa_to_gui))
    osa_process.start()

    # Starts the GUI and wait for responses from OSA
    app = OvertakingSpotAnalyzerGUI(q_gui_to_osa, q_osa_to_gui)
    app.after(100, app.check_for_osa_response)
    app.mainloop()
