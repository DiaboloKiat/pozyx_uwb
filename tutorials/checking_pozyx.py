import pypozyx
import commands

from pypozyx import PozyxSerial, get_first_pozyx_serial_port, UWBSettings
from pypozyx import POZYX_SUCCESS, SingleRegister, EulerAngles, Acceleration
from pypozyx import PozyxConstants
from pypozyx import Coordinates, DeviceCoordinates


#----------------------------------------------------------------------------------------------------------------------
# Finding your serial port
print(pypozyx.get_first_pozyx_serial_port())
print()

# Connecting to the Pozyx checking
serial_port = get_first_pozyx_serial_port()

if serial_port is not None:
    pozyx = PozyxSerial(serial_port)
    #print(pozyx)
    uwb_settings = UWBSettings()
    #print(uwb_settings)
    pozyx.getUWBSettings(uwb_settings)
    # print(uwb_settings)
    print("Connection success!")
else:
    print("No Pozyx port was found")

#----------------------------------------------------------------------------------------------------------------------
# Reading data

# from pypozyx import PozyxSerial, get_first_pozyx_serial_port, POZYX_SUCCESS, SingleRegister, EulerAngles, Acceleration
# initalize the Pozyx as above

# initialize the data container
who_am_i = SingleRegister()
# print(who_am_i) # Will print "0x0"

# get the data, passing along the container
status = pozyx.getWhoAmI(who_am_i)
# print(status) # Will print "1"

# check the status to see if the read was successful. Handling failure is covered later.
# if status == POZYX_SUCCESS:
    # print the container. Note how a SingleRegister will print as a hex string by default.
    # print(who_am_i) # Will print "0x43"

# and repeat
# initialize the data container
acceleration = Acceleration()
# print(acceleration) # Will print "X: 0.0, Y: 0.0, Z: 0.0"

# get the data, passing along the container
pozyx.getAcceleration_mg(acceleration)

# initialize the data container
euler_angles = EulerAngles()
# print(euler_angles) # Will print "Heading: 0.0, Roll: 0.0, Pitch: 0.0"

# get the data, passing along the container
pozyx.getEulerAngles_deg(euler_angles)

#----------------------------------------------------------------------------------------------------------------------
# Writing data

# method 1: making a data object
uwb_channel = SingleRegister(5)
#print(uwb_channel) # Will print "0x5" = hexadecimal
pozyx.setUWBChannel(uwb_channel)

# method 2: or just using the channel number directly
pozyx.setUWBChannel(5)

# both have the same effect!
#-------------------------------

# method 1: making a data object
# this is much more readable
uwb_settings = UWBSettings(channel=5, bitrate=1, prf=2, plen=0x08, gain_db=25.0)
pozyx.setUWBChannel(uwb_channel)

# method 2: using the register values directly
# this isn't readable and also not writable (need to search in depth register documentation)
# pozyx.setUWBSettings([5, 0b10000001, 0x08, 50])

# both still have the same effect, but note how bitrate and prf combine in a register value,
# and gain is doubled when converted to its register contents.

#-------------------------------
# from pypozyx import PozyxSerial, get_first_pozyx_serial_port, POZYX_SUCCESS, SingleRegister, PozyxConstants
# initialize Pozyx as above

# pozyx.setPositionAlgorithm(PozyxConstants.POSITIONING_ALGORITHM_UWB_ONLY)

# new_id = pozyx.getNetworkId(0x1)
# pozyx.setNetworkId(new_id)

# pozyx.setPositioningFilter(PozyxConstant.FILTER_TYPE_MOVING_AVERAGE, 10)

# instead of pozyx.setPositionAlgorithm(PozyxConstants.POSITIONING_ALGORITHM_UWB_ONLY)
pozyx.setPositionAlgorithmNormal()

# instead of pozyx.setPositioningFilter(PozyxConstant.FILTER_TYPE_MOVING_AVERAGE, 10)
pozyx.setPositioningFilterMovingAverage(10)

#----------------------------------------------------------------------------------------------------------------------
# Performing functions

# from pypozyx import ..., Coordinates, DeviceCoordinates

# assume an anchor 0x6038 that we want to add to the device list and immediately save the device list after.
anchor = DeviceCoordinates((0x6a78), 0, Coordinates(5000, 5000, 0))
# print(anchor) # Will print "ID: 0x6A78, flag: 0, X: 5000.0, Y: 5000.0, Z: 0.0"
pozyx.addDevice(anchor)
pozyx.saveNetwork()

# after, we can start positioning. Positioning takes its parameters from the configuration in the tag's
# registers, and so we only need the coordinates.
position = Coordinates()
# print(position) # Will print "X: 0.0, Y: 0.0, Z: 0.0"
pozyx.doPositioning(position)

#----------------------------------------------------------------------------------------------------------------------
# Remote

# let's assume there is another tag present with ID 0x6039
remote_device_id = 0x6a60

# this will read the WHO_AM_I register of the remote tag
who_am_i = SingleRegister()
pozyx.getWhoAmI(who_am_i)
# print(who_am_i) # Will print 0x43

#----------------------------------------------------------------------------------------------------------------------
# Saving writable register data

# Saves the positioning settings
pozyx.savePositioningSettings()
# Saves the device list used for positioning
pozyx.saveNetwork()
# Saves the device's UWB settings
pozyx.saveUWBSettings()

#----------------------------------------------------------------------------------------------------------------------
# Finding out the error

# from pypozyx import PozyxSerial, get_first_pozyx_serial_port, POZYX_SUCCESS, SingleRegister
# initialize Pozyx as above

if pozyx.saveUWBSettings() != POZYX_SUCCESS:
    # this is one way which retrieves the error code
    error_code = SingleRegister()
    pozyx.getErrorCode(error_code)
    print('Pozyx error code: %s' % error_code)
    # the other method returns a descriptive string
    print(pozyx.getSystemError())

remote_id = 0x6a78           # the network ID of the remote device
remote = False               # whether to use the given remote device for ranging
if not remote:
    remote_id = None

print("------------------------------------------------------------")
if remote_id is None:
    for device in [remote_id]:
        pozyx.printDeviceInfo(device)
else:
    for device in [None, remote_id]:
        pozyx.printDeviceInfo(device)
print("------------------------------------------------------------")


print(commands.getstatusoutput("udevadm info -a " + pypozyx.get_first_pozyx_serial_port() + " | grep serial"))



