 print(entry_duration_list)
 import matplotlib.pyplot as plt
 plt.hist(entry_duration_list, bins = 20)
 plt.xlabel('time (s)')
 plt.ylabel('frequency')
 plt.title('beacon stays')
 plt.savefig('hist_%s ' % time_stamp)
 plt.show()

