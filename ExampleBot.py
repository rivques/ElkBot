from tools import  *
from objects import *
from routines import *


#This file is for strategy

class ExampleBot(GoslingAgent):
    def initialize_agent(self):
        super().initialize_agent()
        self.state = None
        self.debug = True
    def run(self):
        my_goal_to_ball, my_ball_distance = (self.ball.location-self.friend_goal.location).normalize(True)
        goal_to_me = self.me.location-self.friend_goal.location
        my_distance = my_goal_to_ball.dot(goal_to_me)

        foe_goal_to_ball, foe_ball_distance = (self.ball.location-self.friend_goal.location).normalize(True)
        try:
            foe_goal_to_foe = self.foes[0].location-self.friend_goal.location
        except IndexError:
            foe_onside = True
        else:
            foe_distance = foe_goal_to_ball.dot(foe_goal_to_foe)
            foe_onside = foe_distance - 200 < foe_ball_distance

        me_onside = my_distance - 200 < my_ball_distance
        close = (self.me.location - self.ball.location).magnitude() < 2000
        have_boost = self.me.boost > 20

        # -1: defensive third, 0: middle third,  1: offensive third
        if -1706 < self.ball.location.y < 1706:
            ball_third = 0 
        elif (self.team and self.ball.location.y > 1706) or (not self.team and self.ball.location.y < -1706):
            ball_third = -1
        else: 
            ball_third = 1

        return_to_goal = False

        need_to_save = False

        bump_opponent = False

        struct = self.get_ball_prediction_struct()
        for pred_slice in struct.slices:
            if side(self.team) * pred_slice.physics.location.y > 5200:
                need_to_save = True
                break

        for pred_slice in struct.slices:
            if side(self.team) * pred_slice.physics.location.y < -5200:
                bump_opponent = True
                break
            

        self.renderer.draw_polyline_3d([pred_slice.physics.location for pred_slice in struct.slices], self.renderer.cyan())

        if len(self.stack) < 1 or (self.state == 'getting boost' and len(self.stack) == 1):
            if self.state == 'getting boost' and len(self.stack) == 1:
                self.pop()
            self.state = None
            if self.kickoff_flag:
                self.push(kickoff())
                self.state = 'kickoff'
            elif need_to_save or ball_third == -1:
                left_field = Vector3(4200*(-side(self.team)), self.ball.location.y + 1000*(-side(self.team)), 0)
                right_field = Vector3(4200*(side(self.team)), self.ball.location.y + 1000*(-side(self.team)), 0)
                self.state = 'need to save'
                team = 1 if self.team == 1 else -1
                targets = {'my_goal': (Vector3(-team * 850, team * 5100, 320), Vector3(team * 850, team * 5100, 320)), 'goal': (self.foe_goal.left_post, self.foe_goal.right_post), 'upfield': (left_field, right_field)}
                shots = find_hits(self, targets)
                if len(shots['goal']) > 0:
                    self.push(shots['goal'][0])
                    self.state = 'shooting (NTS)'
                elif len(shots['upfield']) > 0:
                    self.push(shots['upfield'][0])
                    self.state= 'upfield (NTS)'
                elif len(shots['my_goal']) > 0:
                    self.push(shots['my_goal'][0])
                    self.state = 'not my goal (NTS)'
                else:
                    return_to_goal = True
                    self.state = 'need to save (RTG)'
            elif (close and me_onside) or (not foe_onside and me_onside):
                left_field = Vector3(4200*(-side(self.team)), self.ball.location.y + 1000*(-side(self.team)), 0)
                right_field = Vector3(4200*(side(self.team)), self.ball.location.y + 1000*(-side(self.team)), 0)
                targets = {'goal': (self.foe_goal.left_post, self.foe_goal.right_post), 'upfield': (left_field, right_field)}
                shots = find_hits(self, targets)
                if len(shots['goal']) > 0:
                    self.push(shots['goal'][0])
                    self.state = 'shooting'
                elif len(shots['upfield']) > 0:
                    self.push(shots['upfield'][0])
                    self.state= 'upfield'
                else:
                    return_to_goal = True
                    self.state = 'no shot/upfield (RTG)'
            elif not me_onside and not have_boost and ball_third != -1 and not foe_onside: 
                boosts = [boost for boost in self.boosts if boost.large and boost.active]
                if len(boosts) > 0:
                    closest = boosts[0]
                    for boost in boosts:
                        if (boost.location - self.me.location).magnitude() < (closest.location - self.me.location).magnitude():
                            closest = boost
                    if (self.me.location - closest.location).magnitude() < 2000:
                        self.push(goto_boost(closest, self.ball.location))
                        self.state = 'getting boost'
                    else:
                        state = 'no close boost'
                        return_to_goal = True
                else:
                    return_to_goal = True
                    self.state = 'no boost (RTG)'
            elif not me_onside and foe_onside:
                self.state = 'get onside (RTGw/B)'
                return_to_goal = True
                self.controller.boost = True
            else:
                return_to_goal = True
                self.state = 'RTG'
            
            if return_to_goal and ball_third != -1 and ((self.me.location.y - self.ball.location.y) * side(self.team) > 500) :
                self.state = 'HIT DA BALL'
                relative_target = self.ball.location - self.me.location
                angles = defaultPD(self, self.me.local(relative_target))
                defaultThrottle(self, 2300)
                self.controller.boost = False if abs(angles[1]) > 0.5 or self.me.airborne else self.controller.boost
                self.controller.handbrake = True if abs(angles[1]) > 2.8 else False
            else:
                if not (side(self.team) * self.me.location.y > 5120):
                    relative_target = self.friend_goal.location - self.me.location
                    angles = defaultPD(self, self.me.local(relative_target))
                    defaultThrottle(self, 2300)
                    self.controller.boost = False if abs(angles[1]) > 0.5 or self.me.airborne else self.controller.boost
                    self.controller.handbrake = True if abs(angles[1]) > 2.8 else False
                else:
                    self.state += ' (In goal)'

        if self.debug:
            self.debug_stack()
            for stackitem in self.stack:
                if hasattr(stackitem, 'ball_location'):
                    self.draw_cube_wireframe(stackitem.ball_location, self.renderer.pink())
            self.line(self.friend_goal.location, self.ball.location, (255,255, 255))
            my_point = self.friend_goal.location + (my_goal_to_ball * my_distance)
            self.line(my_point - Vector3(0,0,100),  my_point + Vector3(0,0,100), (0,255,0))
            car_to_ball = 'working!'
            self.renderer.draw_string_2d(10, 30*(self.index + 10), 2, 2, (str(ball_third) + ' ' + str(return_to_goal) + ' ' + str(need_to_save) + ' ' + str(self.state) + ' ' + str(car_to_ball)), self.renderer.white())

    def draw_cube_wireframe(self, center, color, size=75):
        points = []
        for offset in [Vector3(size/2, size/2, size/2), Vector3(size/2, size/2, -size/2), Vector3(size/2, -size/2, size/2), Vector3(size/2, -size/2, -size/2), Vector3(-size/2, size/2, size/2), Vector3(-size/2, size/2, -size/2), Vector3(-size/2, -size/2, size/2), Vector3(-size/2, -size/2, -size/2)]:
            points.append(center + offset)
        for point in points:
            for other_point in points:
                if point == other_point:
                    continue
                if abs((point-other_point).magnitude()) == size:
                    self.renderer.draw_line_3d(point, other_point, color)
        
            