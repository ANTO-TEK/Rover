classdef ROVER_STATE < Simulink.IntEnumType
  enumeration
    IDLE(0)
    FORWARD(1)
    RIGHT(2)
    LEFT(3)
    FORWARD_RIGHT(4)
    FORWARD_LEFT(5)
    BACKWARD(6)
    ALT_BACKWARD(7)
    EMERGENCY(8)
  end
end 