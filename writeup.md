## Your code should compile.
I modified the _CMakeLists.txt_, please check the Readme.


## The Model
The kinematic model was taken from the lectures:
* _x​<sub>t+1​​</sub> = x​<sub>t​​</sub> + v​<sub>t​​</sub> ∗ cos(ψ​<sub>t​​</sub>) ∗ dt_
* _y​<sub>t+1​​</sub> = y​<sub>t​​</sub> + v​​<sub>t *</sub> sin(ψ​<sub>t​​</sub>) ∗ dt_
* _ψ​<sub>t+1​​</sub> = ψ​<sub>t​​</sub> +​ L​f​ ​​​v​<sub>t ​</sub>​​​ ∗ δ ∗ dt_
* _v​<sub>t+1​​</sub> = v​<sub>t​​</sub> + a​<sub>t ​</sub>​ ∗ dt_

.. with _x_ and _y_ as the vehicle's coordinates w.r.t. the vehicle frame, _ψ​_ as the vehicle's orientation w.r.t. the vehicle frame, _v​_ as the vehicle's velocity, _δ_ as the vehicle's steering angle (aka front wheel angle), _v_ as velocity and _a​_ as acceleration.

## Timestep Length and Elapsed Duration (N & dt)
I chose them empirically. If done right, they should not have a big impact on the performance, should they? See the [Open Questions](/#open_questions) section.


## Polynomial Fitting and MPC Preprocessing
**Waypoints** .. were transformed to the vehicle frame such they are relative to the car.

**VehicleState** .. is forward-simulated, see the next section. Despite that, the input velocity comes in _MPH_, the input steering with the wrong sign, both wasn't stated and therefore very confusing.


## Model Predictive Control with Latency
The vehicle model is forward simulated by the same motion model that is used within the MPC kinematic model (_CVTR_ right?). The duration of this prediction is the latency + execution time of the whole algorithm, wich was empirically measured and set to 0.12 in total. Without this latency-compensation, the vehicle was hard to control and oscillated a lot. After implementing, higher speed were possible.

However, I was wondering why one should program the same motion model again when this is already done in the MPC kinematic model. See [this discussion](https://discussions.udacity.com/t/how-to-incorporate-latency-into-the-model/257391/94) to check my approach of dealing with latency within the optimization itself.


## The vehicle must successfully drive a lap around the track.
Works on my machine ;)


## Open Questions
* Why is the choice of `dt` and `N` so crucial and influential to the overall performance?
  * observation: changing _dt_ or N leads to very different behavior
  * since I already accounted for latency with forward simulating the kinematic model, I'd assume
   that I can choose dt and N as I wish without much impact on the behavior
* Why is my trajectory, i.e. the green line, diverting from the reference line, i.e. the yellow one, in the end, but close in the beginning?
  * I guess due to the wrong `cte(x)` function ([ref](https://discussions.udacity.com/t/how-does-the-mpc-manipulate-this-90-degree-turn-example/273950))
* How would I model obstacles with MPC resp. boundary regions like stop-lines or goal poses in this form of differentiable functions? I don't see it with _CppAD_ and this optmization scheme that requires _twice continuous differentiable functions_ for constraints and cost-terms. If it can't be done, of what use is this optimization scheme in the real world scenario? ([ref](https://discussions.udacity.com/t/dynamic-obstacles-with-mpc-and-ipopt/407334))
* Is this now a Linear MPC (_LMPC_) or a non-linear MPC (_NMPC_)?
* Where is the vehicle frame? Center of gravity, front-axle center, rear-axle center?
