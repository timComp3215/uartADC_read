'''
Read ADC samples from board (as ASCII)
Convert to voltage
Display time series data
Output data to csv file
'''

#Import libraries
import serial
import csv
import matplotlib.pyplot as plt

#Set length of signal
SAMPLES = 2048

#set name of output file
output_file = 'adc_output.csv'

#Sampling frequency - purely for graphs to be correctly scaled
fs = 5000

#Connect to serial channel and send input
s = serial.Serial('COM4', 9600)

print "Reading messages from board.."

#Read messages
values = []
float_values = []
    
n=0
for n in range(SAMPLES):

    if (n == (SAMPLES/4)):
        print "25%..."
    elif (n == (SAMPLES/2)):
        print "50%..."
    elif (n == (3*SAMPLES/4)):
        print "75%..."
    msg = s.readline()

    #receivedMsg = int(msg)

    #values.append((receivedMsg/16384) * 3.3)
    values.append(int(msg))

#Close serial channel
s.close()
print "Reading finished"

for n in range(0, SAMPLES):
    float_values.append((float(values[n])/16384) * 3.3)

#print values
#Store output in csv
print "Write output to " + output_file
with open(output_file, 'wb') as fout:
    writer = csv.writer(fout)
    for n in range(0, SAMPLES):
        #Integer must be inside [] brackets
        writer.writerow([values[n]])

    fout.close()


#Set up x axes variables for plotting
time = []

for n in range(0, SAMPLES):
    time.append(n*(1/float(fs)))

#Plot
fig = plt.figure(1)

#Input signal
plt.plot(time, float_values, linewidth=0.5)
plt.ylabel('Voltage')
plt.xlabel('Time (s)')
plt.title('Time based data')
#Get rid of white space on x axis
plt.autoscale(enable=True, axis='x', tight=True)
#Put ticks on outside for bottom and left axes
ax = plt.gca()
ax.tick_params(direction='out')
ax.tick_params(bottom=True, left=True, top=False, right=False)
ax.set_ylim([0,3.5])

#Background colour
fig.patch.set_facecolor('white')

#Space subplots properly
plt.tight_layout()

#Display
print "Display graphs"
print "Please close graphs before continuing"
plt.show()

