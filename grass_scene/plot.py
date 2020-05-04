# Plotting

import matplotlib.pyplot as plt


def plot1(cum_in1,cum_in2,entry_duration_graph,sham_entry_duration_graph):
    """plot histogram"""


    plt.style.use('ggplot')
    fig, ax = plt.subplots(1, 2, figsize=(18, 9), sharey=True, sharex=True)
    fig.text(0.30, 0.8, 'Time in beacon: %0.0f ' % cum_in1, bbox=dict(facecolor='green', alpha=.5),
             weight="bold")
    fig.text(0.75, 0.8, 'Time in SHAM beacon: %0.0f ' % cum_in2, bbox=dict(facecolor='cyan', alpha=.5),
             weight="bold")
    ax[0].hist(entry_duration_graph, bins=20, color='olive')
    ax[0].set(xlabel='time (s)', ylabel='frequency', title='beacon stays')
    ax[0].set_yscale('log')
    ax[1].hist(sham_entry_duration_graph, bins=20, color='teal')
    ax[1].set_yscale('log')
    ax[1].set(xlabel='time (s)', ylabel='frequency', title='SHAM beacon stays')

    fig.canvas.set_window_title('Beacon stays')
    fig.tight_layout()

    return fig


def plot2(ratxX,ratyY,ratz):

    fig2 = plt.figure(figsize=(18, 9))
    ax = fig2.gca(projection='3d')
    ax = fig2.add_subplot(1, 1, 1, projection='3d')
    ax.set(xlabel='x_position', ylabel='Y-position', zlabel='Height', title='beacon stays')
    ax.plot(ratxX, ratyY, ratz)
    ax.view_init(-65, 70)

    return fig2


def plot3(ratxX,ratyY,speed,entry_timestamp_list,feed_counts,distance):

    fig3, ax1 = plt.subplots(1, 3, figsize=(18, 9))
    ax1[0].hist2d(ratxX, ratyY, bins=20, )
    ax1[0].set(xlabel='X', ylabel='Y', title='Movement histogram', )
    ax1[1].plot(speed)
    ax1[1].set(xlabel='time', ylabel='speed', title='Velocity graph')
    ax1[2].hist(entry_timestamp_list, bins=40, color='gold')
    ax1[2].set(xlabel='time point', ylabel='frequency', title='beacon entries')
    fig3.text(0.75, 0.8, 'Number of pellets: %0.0f ' % feed_counts, bbox=dict(facecolor='yellow', alpha=.5),
              weight="bold")
    fig3.text(0.45, 0.8, 'Distance traveled: %0.2f meters' % (distance),
              bbox=dict(facecolor='olive', alpha=.5), weight="bold")
    fig3.tight_layout()

#Can be uncommented to show all graphs when finished.
    #plt.show()

    return fig3

