== Open Questions
* Why is the choice of dt and N so crucial and influential to the overall performance?
 - since I already accounted for latency with forward simulating the kinematic model, I'd assume
   that I can choose dt and N as I wish without much impact on the behavior
* Why is my trajectory, i.e. the green line, diverting from the reference line, i.e. the yellow one, in the end, but close in    the beginning?
* How would I model obstacles with MPC resp. boundary regions like stop-lines or goal poses in this form of differentiable functions?
* Is this now a Linear MPC (LMPC) or a non-linear MPC (NMPC)?