#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg  import Pose
import math

class point:
	def __init__(self,x,y):
		self.x=x
		self.y=y

# lista pnktow do ktorych zolw ma dotrzec
points_list=[point(9,9), point(1,2), point(5.4,10), point(9.9,0.8), point(0.2, 8.75), point(0.2,0.2)] 
point_index=0 # indeks punktu docelowego
r=100 # odleglosc zolwia od punktu docelowego (poczatkowo ustawiam dowolna wartosc, potem zostanie ona wyznaczona)
K_lin=8 # wspolczynnik do wyznaczania predkosci liniowej
K_ang=7 # wspolczynnik do wyznaczania predkosci katowej
precision=0.005
new_vel = Twist()

# wyznaczanie docelowej wartosci theta oraz odleglosci zolwia od punktu docelowego w danym momencie
# T-aktualne polozenie zolwia, P-punkt docelowy
def calculate_needed_theta(T,P):
	global r
	delta_x=P.x-T.x
	delta_y=P.y-T.y
	r=math.sqrt(delta_x*delta_x+delta_y*delta_y)
	if r<precision: # jesli zolw znajduje sie juz w punkcie docelowym, to nie trzeba wyznaczac wartosci theta
		return 10
	alfa=math.asin(delta_y/r)
	if P.x<T.x:
		if alfa>0:
			alfa=math.pi-alfa
		else:
			alfa=-math.pi-alfa
	return alfa

def callback(pose):
	global new_vel
	global point_index
	new_vel = Twist()

	if point_index>=len(points_list): # zolw dotarl juz do wszystkich punktow z listy
		new_vel.linear.x = 0.0
		new_vel.angular.z = 0.0
		return

	needed_theta=calculate_needed_theta(point(pose.x,pose.y), points_list[point_index])
	if r<precision: # zolw dotarl do punktu docelowego
		print("\nZolw dotarl do"),
		print(str(point_index+1)),
		print(" punktu z listy")
		rospy.loginfo("Pozycja x: %8.2f",pose.x)
		rospy.loginfo("Pozycja y: %8.2f",pose.y)
		rospy.loginfo("Pozycja theta: %8.2f",pose.theta)
		point_index=point_index+1 # indeks nowego punktu docelowego
		return

	if abs(pose.theta-needed_theta) > precision:
		# obrot
		new_vel.linear.x = 0.0
		new_vel.angular.z=K_ang*(needed_theta-pose.theta)
	else:
		# ruch do przodu
		new_vel.linear.x = K_lin*r
		new_vel.angular.z=0.0

if __name__== "__main__":
	global new_vel
	new_vel = Twist()
	rospy.init_node('zad2', anonymous=True)
	print("ready")
	pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
	rospy.Subscriber( '/turtle1/pose' , Pose, callback)
	
	rate=rospy.Rate(10) # 10Hz
	while not rospy.is_shutdown():
		pub.publish(new_vel) # wyslanie predkosci zadanej
		rate.sleep()

	print("END")
