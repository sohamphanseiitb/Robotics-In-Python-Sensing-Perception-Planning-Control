# Here we implement a succesive approximation AD converter
# The converter is implemented as a class, and the class has a method to convert a voltage to a digital value

import numpy as np
import matplotlib.pyplot as plt

class SApproxADConverter():

    def __init__(self, n_bits, ref_voltage):
        
        self.n_bits = n_bits
        self.bit_array = np.zeros(n_bits)
        self.ref_voltage = ref_voltage

    def digital_to_analog(self):

        # convert the given bit array into an analog voltage
        voltage = 0

        for i in range(self.n_bits):
            voltage += self.bit_array[i] * 2**i
        
        return voltage
    
    def convert(self):

        # check all bits
        for i in range(self.n_bits-1, -1, -1):

            # start from the most significant bit

            # set bit equal to 1
            self.bit_array[i] = 1

            # compute voltage
            voltage = self.digital_to_analog()

            # check whether it is grea5ter than the reference voltage
            if voltage > self.ref_voltage:
                self.bit_array[i] = 0
        
        return self.bit_array
    
    

# test the class with an example
# number of bits in the converter
n = 4
readings = 10
refv = 5

# reference voltage
xaxis = np.arange(readings)
ref_voltage = np.random.rand(readings) * refv
converted_voltage = np.zeros(readings)
counter = 0

for each in ref_voltage:
    # create an instance of the class
    converter = SApproxADConverter(n, each)
    
    # perform the conversion & store the converted voltage
    store_var = converter.convert()

    # convert bit array to analog voltage
    converted_voltage[counter] = converter.digital_to_analog()

    counter += 1

# plot
plt.scatter(xaxis, ref_voltage, label='Reference Voltage')
plt.scatter(xaxis, converted_voltage, label='Converted Voltage')
plt.vlines(xaxis, ref_voltage, converted_voltage, color='gray', linestyle='dashed')
plt.xlabel('Readings')
plt.ylabel('Voltage')
plt.title('Succesive Approximation AD Converter')
plt.legend()
plt.show()



