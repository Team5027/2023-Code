
/*package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

public class ArmStateMachine {
            private enum ArmState {
                IDLE, MOVING_UP, MOVING_DOWN,
            }
        
            private ArmState state;
            private WPI_VictorSPX armMotor; // Replace "WPI_TalonSRX" with your motor controller type
        
            public ArmStateMachine(WPI_VictorSPX armMotor) {
                this.armMotor = armMotor;
                state = ArmState.IDLE;
            }
        
            public void moveToIdle() {
              switch (state) {
                  case MOVING_UP:
                  case MOVING_DOWN:
                      armMotor.set(ControlMode.PercentOutput, 0.0);
                      state = ArmState.IDLE;
                      break;
                  default:
                      break;
              }
          }
          
          public void moveToMovingUp() {
              switch (state) {
                  case IDLE:
                      armMotor.set(ControlMode.PercentOutput, -0.3); // Replace "0.5" with the desired motor speed
                      state = ArmState.MOVING_UP;
                      break;
                  case MOVING_DOWN:
                      armMotor.set(ControlMode.PercentOutput, 0.0); // Stop the motor before changing directions
                      state = ArmState.IDLE;
                      moveToMovingUp(); // Recursive call to immediately start moving up
                      break;
                  default:
                      break;
              }
            }
            public void moveToMovingDown() {
            switch (state) {
                case IDLE:
                    armMotor.set(ControlMode.PercentOutput, 0.3);
                    state = ArmState.MOVING_DOWN;
                    
                    break;
                case MOVING_UP:
                    armMotor.set(ControlMode.PercentOutput, 0.0); // Stop the motor before changing directions
                    state = ArmState.IDLE;
                    moveToMovingDown(); // Recursive call to immediately start moving up
                    break;
                default:
                    break;
            }
            
        }

            public void periodic() {
            }
        
        
    }

    */
   


  
    


