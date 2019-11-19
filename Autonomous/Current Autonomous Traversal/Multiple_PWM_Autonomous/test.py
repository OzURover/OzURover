from autonomy_controller import AutonomyController
from point import Point

p2 = Point(48,0)
p3 = Point(73,-25)
autonomy_calculator = AutonomyController(p3, p2, 0)
line_angle = autonomy_calculator.line_angle
target_angle = autonomy_calculator.target_angle
target_distance = autonomy_calculator.target_distance

print("Line angle:", line_angle)
print("Target angle:", target_angle)
print("Target distance:", target_distance)
