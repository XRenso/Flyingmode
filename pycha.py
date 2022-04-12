import math
import rospy
from clover import srv
from std_srvs.srv import Trigger
from sensor_msgs.msg import Range
from mavros_msgs.srv import CommandBool
from mavros_msgs.srv import SetMode


rospy.init_node('moving_platform_land')




set_mode = rospy.ServiceProxy('mavros/set_mode', SetMode)
arming = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)
get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
set_position = rospy.ServiceProxy('set_position', srv.SetPosition)
land = rospy.ServiceProxy('land', Trigger)
set_effect = rospy.ServiceProxy('led/set_effect', srv.SetLEDEffect)

tar_aruco = 'aruco_141'
arco = input('aruco_')
near_aruco = 'aruco_' + arco
z_base = 1
z_safe = 0.5
z_min = 0.2
dz = 0.1
xy_safe = 0.1
v_safe = 0.1
range = 0
landed = False



def navigate_wait(x=0, y=0, z=0, yaw=float('nan'), speed=0.5, frame_id='', auto_arm=False, tolerance=0.2):
    navigate(x=x, y=y, z=z, yaw=yaw, speed=speed, frame_id=frame_id, auto_arm=auto_arm)

    while not rospy.is_shutdown():
        telem = get_telemetry(frame_id='navigate_target')
        if math.sqrt(telem.x ** 2 + telem.y ** 2 + telem.z ** 2) < tolerance:
            break
        rospy.sleep(0.2)

def land_wait():
    land()
    while get_telemetry().armed:
        rospy.sleep(0.2)




def disarm():
    global landed
    arming(False)
    set_mode(custom_mode='STABILIZED')
    landed=True
    land()



def range_callback(msg):
    global landed, range
    range = msg.range
    if range < z_min:
        disarm()


def main():
    global range, landed,z_base
    set_effect(effect='blink', r=255, g=255, b=255)
    rospy.sleep(1)
    set_effect(r=0,b=0,g=0)
    z_base = 1
    navigate_wait(z=1.5,frame_id='body',auto_arm=True)
    navigate_wait(z=1.5,frame_id='aruco_134')
    navigate_wait(z=1.5,frame_id=near_aruco)
    rospy.sleep(0.4)
    navigate_wait(z=1,frame_id=tar_aruco)
    rospy.Subscriber('rangefinder/range', Range, range_callback)
    set_effect(r=0,g=0,b=255)
    while not rospy.is_shutdown() and not landed:
        t = get_telemetry(frame_id=tar_aruco)
        t1 = get_telemetry(frame_id='aruco_map')
        set_position(x=0,y=0,z=z_base,frame_id=tar_aruco)
        v = (t1.vx**2+t1.vy**2)**0.5
        d = (t.x**2 + t.y**2)**0.5

        if z_base > z_safe:
            z_base -= dz
            set_effect(r=255,g=255,b=0)


        if d <= xy_safe and v <= v_safe and t.z <= z_safe:
            # if get_telemetry.armed():
            #     arming(False)
            #     land()
            disarm()
            landed = True

        rospy.sleep(0.1)

if get_telemetry().armed and landed == True:
    arming(False)
    set_mode(custom_mode='STABILIZED')

if __name__ == "__main__":
    set_effect(r=0,b=0,g=0)
    main()
