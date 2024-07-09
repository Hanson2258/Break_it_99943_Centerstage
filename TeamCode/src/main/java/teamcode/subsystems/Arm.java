package teamcode.subsystems;

import androidx.annotation.NonNull;

import java.util.ArrayList;
import java.util.Locale;

import TrcCommonLib.trclib.TrcDbgTrace;
import TrcCommonLib.trclib.TrcEvent;
import TrcCommonLib.trclib.TrcExclusiveSubsystem;
import TrcCommonLib.trclib.TrcMotor;
import TrcCommonLib.trclib.TrcRobot;
import TrcCommonLib.trclib.TrcSensor;
import TrcCommonLib.trclib.TrcTaskMgr;
import TrcCommonLib.trclib.TrcTimer;
import TrcFtcLib.ftclib.FtcDistanceSensor;
import TrcFtcLib.ftclib.FtcMotorActuator;
import TrcFtcLib.ftclib.FtcServo;
import TrcFtcLib.ftclib.FtcServoActuator;
import teamcode.RobotParams;

public class Arm implements TrcExclusiveSubsystem
{
		private final String moduleName = Arm.class.getSimpleName(); // Get name of the class

		/**
		 * Action list for the Arm Rotator and Extender
		 */
		private enum ActionType
		{
				RotatorSetPosition,
				RotatorSetPower,
				RotatorSetPidPower,
				ExtenderSetPosition,
				ExtenderSetPower,
				ExtenderSetPidPower
		} // enum ActionType

		/**
		 * This class contains the parameters for Actions.
		 */
		private static class ActionParams
		{
				ActionType actionType = null;
				TrcEvent actionEvent = null;
				double delay = 0.0;
				double pos = 0.0;
				double powerLimit = 1.0;
				double power = 0.0;
				double duration = 0.0;
				double minPos = 0.0;
				double maxPos = 0.0;
				TrcEvent completionEvent = null;
				double expiredTime = 0.0;

				/**
				 * Set the position params.
				 *
				 * @param actionType specifies the type of action.
				 * @param actionEvent specifies the Action Event.
				 * @param delay specifies the delay before moving in seconds.
				 * @param pos specifies the position to move to.
				 * @param powerLimit specifies the max power limit.
				 * @param completionEvent specifies the event to signal when operation is completed, can be null if not provided.
				 * @param expiredTime specifies the timeout.
				 */
				void setPositionParams(
								ActionType actionType, TrcEvent actionEvent, double delay, double pos, double powerLimit,
								TrcEvent completionEvent, double expiredTime)
				{
						this.actionType = actionType;
						this.actionEvent = actionEvent;
						this.delay = delay;
						this.pos = pos;
						this.powerLimit = powerLimit;
						this.completionEvent = completionEvent;
						this.expiredTime = expiredTime;
				} // setPositionParams

				/**
				 * Set the power params.
				 *
				 * @param actionType specifies the type of action.
				 * @param actionEvent specifies the Action Event.
				 * @param delay specifies the delay before moving in seconds.
				 * @param power specifies the percentage power (range -1.0 to 1.0).
				 * @param duration specifies the duration in seconds to run the motor and turns it off afterwards, 0.0 if not provided.
				 * @param completionEvent specifies the event to signal when operation is completed, can be null if not provided.
				 */
				void setPowerParams(
								ActionType actionType, TrcEvent actionEvent, double delay, double power, double duration,
								TrcEvent completionEvent)
				{
						this.actionType = actionType;
						this.actionEvent = actionEvent;
						this.delay = delay;
						this.power = power;
						this.duration = duration;
						this.completionEvent = completionEvent;
				} // setPowerParams

				/**
				 * Set the PID power params.
				 *
				 * @param actionType specifies the type of action.
				 * @param actionEvent specifies the Action Event.
				 * @param power specifies the power to use.
				 * @param minPos specifies the min position.
				 * @param maxPos specifies the amx position.
				 */
				void setPidPowerParams(
								ActionType actionType, TrcEvent actionEvent, double power, double minPos, double maxPos)
				{
						this.actionType = actionType;
						this.actionEvent = actionEvent;
						this.power = power;
						this.minPos = minPos;
						this.maxPos = maxPos;
				} // setPidPowerParams

				// @TODO Change to NOT use string formats after speed test
				/**
				 * Display the param details.
				 *
				 * @return string format of each param.
				 */
				@NonNull
				@Override
				public String toString()
				{
						return String.format(
										Locale.US,
										"\n(\tactionType=%s\n" +
														"\tactionEvent=%s\n" +
														"\tdelay=%.3f\n" +
														"\tpos=%.1f\n" +
														"\tpowerLimit=%.1f\n" +
														"\tpower=%.1f\n" +
														"\tduration=%.1f\n" +
														"\tposRange=%.1f/%.1f\n" +
														"\tevent=%s\n" +
														"\texpiredTime=%.3f\n)",
										actionType, actionEvent, delay, pos, powerLimit, power, duration, minPos, maxPos, completionEvent,
										expiredTime);

//						return "\n(\tactionType=" + actionType + "\n" +
//										  "\nactionEvent=" + actionEvent + "\n" +
//										  "\tdelay=" + delay + "\n" +
//										  "\tpos=" + pos + "\n" +
//  										"\tpowerLimit=" + powerLimit + "\n" +
//	  									"\tpower=" + power + "\n" +
//		  								"\tduration=" + duration + "\n" +
//			  							"\tposRange=" + minPos + "/" + maxPos + "\n" +
//				  						"\tevent=" + completionEvent + "\n" +
//					  					"\texpiredTime=" + expiredTime + "\n" +
//						  				")";
				} // toString
		} // class ActionParams


		// Debugging/ Tracing tools
		private final ArrayList<TrcEvent> pendingEventCallbacks = new ArrayList<>();
		private final TrcDbgTrace tracer;


		// Rotator subsystem
		public final TrcMotor rotator;
		private double elevatorPrevPrintedPower = 0.0;
		// Extender subsystem
		public final TrcMotor extender;
		private double armPrevPrintedPower = 0.0;
		// Wrist subsystem
		public final FtcServo wrist;
		public final FtcDistanceSensor wristSensorTop;
		public final FtcDistanceSensor wristSensorBot;
		private final TrcTaskMgr.TaskObject wristSensorTaskObj;
		private TrcEvent wristCompletionEvent;


		/**
		 * Constructor: Creates an instance of the object.
		 */
		public Arm()
		{
				tracer = new TrcDbgTrace(); // Initializing Debug Tracer

				// Rotator subsystem
				if (RobotParams.Preferences.useArmRotator)
				{
						FtcMotorActuator.Params armParams = new FtcMotorActuator.Params()
										.setMotorInverted(RobotParams.ARM_ROTATOR_MOTOR_INVERTED)
										.setVoltageCompensationEnabled(RobotParams.ARM_ROTATOR_VOLTAGE_COMP_ENABLED)
										.setPositionScaleAndOffset(RobotParams.ARM_ROTATOR_DEG_PER_COUNT, RobotParams.ARM_ROTATOR_OFFSET)
										.setPositionPresets(RobotParams.ARM_ROTATOR_PRESET_TOLERANCE, RobotParams.ARM_ROTATOR_PRESETS);
						rotator = new FtcMotorActuator(RobotParams.HWNAME_ARM_ROTATOR, armParams).getActuator();
						rotator.setPositionPidCoefficients(
										RobotParams.ARM_ROTATOR_KP, RobotParams.ARM_ROTATOR_KI, RobotParams.ARM_ROTATOR_KD,
										RobotParams.ARM_ROTATOR_KF, RobotParams.ARM_ROTATOR_IZONE);
						rotator.setPositionPidTolerance(RobotParams.ARM_ROTATOR_TOLERANCE);
						rotator.setStallDetectionEnabled(
										RobotParams.ARM_ROTATOR_STALL_DETECTION_DELAY, RobotParams.ARM_ROTATOR_STALL_DETECTION_TIMEOUT,
										RobotParams.ARM_ROTATOR_STALL_ERR_RATE_THRESHOLD);
						rotator.setTraceLevel(TrcDbgTrace.MsgLevel.INFO, false, false, null);
				}
				else
				{
						rotator = null;
				}
				// Extender subsystem
				if (RobotParams.Preferences.useArm)
				{
						FtcMotorActuator.Params elevatorParams = new FtcMotorActuator.Params()
										.setMotorInverted(RobotParams.ARM_EXTENDER_MOTOR_INVERTED)
										.setVoltageCompensationEnabled(RobotParams.ARM_EXTENDER_VOLTAGE_COMP_ENABLED)
										.setPositionScaleAndOffset(RobotParams.ARM_EXTENDER_INCHES_PER_COUNT, RobotParams.ARM_EXTENDER_OFFSET)
										.setPositionPresets(RobotParams.ARM_EXTENDER_PRESET_TOLERANCE, RobotParams.ARM_EXTENDER_PRESETS);
						extender = new FtcMotorActuator(RobotParams.HWNAME_ARM_EXTENDER, elevatorParams).getActuator();
						extender.setSoftwarePidEnabled(true);
						extender.setPositionPidCoefficients(
										RobotParams.ARM_EXTENDER_KP, RobotParams.ARM_EXTENDER_KI, RobotParams.ARM_EXTENDER_KD,
										RobotParams.ARM_EXTENDER_KF, RobotParams.ARM_EXTENDER_IZONE);
						extender.setStallDetectionEnabled(
										RobotParams.ARM_EXTENDER_STALL_DETECTION_DELAY, RobotParams.ARM_EXTENDER_STALL_DETECTION_TIMEOUT,
										RobotParams.ARM_EXTENDER_STALL_ERR_RATE_THRESHOLD);
						extender.setPositionPidPowerComp(this::armRotatorGetPowerComp);
						extender.setTraceLevel(TrcDbgTrace.MsgLevel.INFO, false, false, null);
				}
				else
				{
						extender = null;
				}
				// Wrist subsystem.
				if (RobotParams.Preferences.useWrist)
				{
						FtcServoActuator.Params wristParams = new FtcServoActuator.Params()
										.setServoInverted(RobotParams.WRIST_SERVO_INVERTED);
						wrist = new FtcServoActuator(RobotParams.HWNAME_WRIST, wristParams).getActuator();
						if (RobotParams.Preferences.useWristSensor)
						{
								wristSensorTaskObj = TrcTaskMgr.createTask("WristSensorTask", this::wristSensorTask);

								wristSensorTop = new FtcDistanceSensor(RobotParams.HWNAME_WRIST_LASER_TOP);
								wristSensorBot = new FtcDistanceSensor(RobotParams.HWNAME_WRIST_LASER_BOT);
						}
						else
						{
								wristSensorTop = null;
								wristSensorBot = null;
								wristSensorTaskObj = null;
						}
						wrist.setTraceLevel(TrcDbgTrace.MsgLevel.INFO);
				}
				else
				{
						wrist = null;
						wristSensorTop = null;
						wristSensorBot = null;
						wristSensorTaskObj = null;
				}
		} // Arm

		/**
		 * This method returns the module name.
		 *
		 * @return module name.
		 */
		@NonNull
		@Override
		public String toString()
		{
				return moduleName;
		} // toString


		/**
		 * This method cancels the arm rotator and extender operation if there is any.
		 *
		 * @param owner specifies the owner ID to check if the caller has ownership of the subsystems.
		 */
		public void cancel(String owner)
		{
				tracer.traceInfo(moduleName, "owner=" + owner);
				if (hasOwnership(owner))
				{
						rotator.cancel();
						extender.cancel();

						// Check if there are any pending event callbacks and remove them.
						synchronized (pendingEventCallbacks)
						{
								if (!pendingEventCallbacks.isEmpty())
								{
										for (TrcEvent event: pendingEventCallbacks)
										{
												event.setCallback(null, null);
										}
										pendingEventCallbacks.clear();
								}
						}
						releaseExclusiveAccess(owner);
				}
		} // cancel

		/**
		 * This method cancels the arm rotator and extender operation if there is any.
		 */
		public void cancel()
		{
				cancel(null);
		} // cancel


		/**
		 * This method removes the event from the Pending Event Callback list if it is in there.
		 *
		 * @param event specifies the event to be removed from the list.
		 */

		private void removePendingEventCallback(TrcEvent event)
		{
				if (event != null)
				{
						synchronized (pendingEventCallbacks)
						{
								pendingEventCallbacks.remove(event);
						}
				}
		} // removePendingEventCallback


		/**
		 * This method sets up the action callback to finish the action when it's safe to do so.
		 *
		 * @param actionParams specifies the action parameters to perform.
		 * @param actionEvent specifies the event that signals the callback for completion.
		 */
		private void setActionCallback(ActionParams actionParams, TrcEvent actionEvent)
		{
				actionEvent.setCallback(this::performAction, actionParams);
				pendingEventCallbacks.add(actionEvent);
		} // setActionCallback


		/**
		 * This method zero calibrates the arm rotator and extender.
		 * Note: This assumes the arm is at safe position zero calibration won't happen.
		 *
		 * @param owner specifies the owner ID to check if the caller has ownership of the motor.
		 */
		public void zeroCalibrate(String owner)
		{
				tracer.traceInfo(moduleName, "owner=" + owner);
				cancel(owner);
				TrcEvent completionEvent = new TrcEvent(moduleName + ".zeroCalComplete");
				TrcEvent releaseOwnershipEvent = acquireOwnership(owner, completionEvent, tracer);

				if (validateOwnership(owner))
				{
						// Do zero calibration only if arm is tucked in @TODO Check if using get position will be fine, or if units need to be scaled
						if (extender != null && rotator != null &&
										rotator.getPosition() - RobotParams.ARM_ROTATOR_TOLERANCE <= RobotParams.ARM_ROTATOR_LOAD_DEG &&
										extender.getPosition() - RobotParams.ARM_EXTENDER_TOLERANCE <= RobotParams.ARM_EXTENDER_LOAD_POS)
						{
								if (wrist != null)
								{
										// Holding wrist in loading position while zero calibrating
										wrist.setPosition(RobotParams.WRIST_COLLECTING_POS);
								}
								// Holding arm in loading position while zero calibrating
								rotator.setPosition(null, 0.0, RobotParams.ARM_ROTATOR_LOAD_DEG, true, RobotParams.ARM_ROTATOR_POWER_LIMIT, null, 0.0);
								extender.setPosition(null, 0.0, RobotParams.ARM_EXTENDER_LOAD_POS, true, RobotParams.ARM_EXTENDER_POWER_LIMIT, null, 0.0);

								// Enable stall protection in case the limit switch is malfunctioning so we can still zero calibrate on stall
								rotator.setStallProtection(RobotParams.ARM_ROTATOR_CAL_POWER, 0.1, 0.2, 0.5);
								extender.setStallProtection(RobotParams.ARM_EXTENDER_CAL_POWER, 0.1, 0.2, 0.5);
								completionEvent.setCallback(this::zeroCalCompleted, null);
								rotator.zeroCalibrate(
												RobotParams.ARM_ROTATOR_CAL_POWER,
												releaseOwnershipEvent != null? releaseOwnershipEvent: completionEvent);
								extender.zeroCalibrate(
												RobotParams.ARM_EXTENDER_CAL_POWER,
												releaseOwnershipEvent != null? releaseOwnershipEvent: completionEvent);
						}
				}
		} // zeroCalibrate

		/**
		 * This method is called after zero calibration is done so we can turn off stall protection.
		 *
		 * @param context not used.
		 */
		// @TODO Add limit switches
		private void zeroCalCompleted(Object context)
		{
				rotator.setStallProtection(0.0, 0.0, 0.0, 0.0);
				extender.setStallProtection(0.0, 0.0, 0.0, 0.0);
//				tracer.traceInfo(
//								moduleName, "Zero Calibrate completed (rotator - lowerLimitSw=" + rotator.isLowerLimitSwitchActive() + ").");
//				tracer.traceInfo(
//								moduleName, "Zero Calibrate completed (extender - lowerLimitSw=" + extender.isLowerLimitSwitchActive() + ").");
		} // zeroCalCompleted


		/**
		 * This method is a callback to perform the specified action when it is safe to do so.
		 *
		 * @param context specifies the action parameters.
		 */
		private void performAction(Object context)
		{
				ActionParams actionParams = (ActionParams) context;
				double delay, timeout;

				switch (actionParams.actionType)
				{
						// Extender
						case ExtenderSetPosition:
								tracer.traceInfo(moduleName, "actionParams=" + actionParams);
								// Extend the arm only if the arm is already at or has reached safe position.
								// actionParams.actionEvent is null if arm is already at safe position.
								if (actionParams.actionEvent == null || actionParams.actionEvent.isSignaled())
								{
										// Delay is only applicable for first movement. If arm has moved, it already applied delay
										delay = actionParams.actionEvent == null ? actionParams.delay : 0.0;
										timeout = actionParams.expiredTime == 0.0 ? 0.0 : actionParams.expiredTime - TrcTimer.getCurrentTime();
										// Move elevator to desired position
										extender.setPosition(
														null, delay, actionParams.pos, true, actionParams.powerLimit, actionParams.completionEvent,
														timeout);
								}
								break;

						case ExtenderSetPower:
								// Trace only if power has changed
								if (actionParams.power != elevatorPrevPrintedPower)
								{
										tracer.traceInfo(moduleName, "actionParams=" + actionParams);
										elevatorPrevPrintedPower = actionParams.power;
								}
								// Extend the arm only if the arm is already at or has reached safe position.
								// actionParams.actionEvent is null if arm is already at safe position.
								if (actionParams.actionEvent == null || actionParams.actionEvent.isSignaled())
								{
										// Delay is only applicable for first movement. If arm has moved, it already applied delay.
										delay = actionParams.actionEvent == null ? actionParams.delay : 0.0;
										// Move elevator with desired power.
										extender.setPower(
														null, delay, actionParams.power, actionParams.duration, actionParams.completionEvent);
								}
								break;

						case ExtenderSetPidPower:
								// Trace only if power has changed
								if (actionParams.power != elevatorPrevPrintedPower)
								{
										tracer.traceInfo(moduleName, "actionParams=" + actionParams);
										elevatorPrevPrintedPower = actionParams.power;
								}
								// Move the elevator only if the arm is already at or has reached safe position.
								// actionParams.actionEvent is null if arm is already at safe position.
								if (actionParams.actionEvent == null || actionParams.actionEvent.isSignaled())
								{
										// Move elevator with desired power with PID control
										extender.setPidPower(null, actionParams.power, actionParams.minPos, actionParams.maxPos, true);
								}
								break;


						// Rotator
						case RotatorSetPosition:
								tracer.traceInfo(moduleName, "actionParams=" + actionParams);
								// Move the arm only if the elevator is already at or has reached safe height.
								// actionParams.actionEvent is null if elevator is already at safe height.
								if (actionParams.actionEvent == null || actionParams.actionEvent.isSignaled())
								{
										// Delay is only applicable for first movement. If elevator has moved, it already applied delay
										delay = actionParams.actionEvent == null ? actionParams.delay : 0.0;
										timeout = actionParams.expiredTime == 0.0 ? 0.0 : actionParams.expiredTime - TrcTimer.getCurrentTime();
										// Make sure wrist is at appropriate position
										if (actionParams.pos >= RobotParams.ARM_ROTATOR_SAFE_DEG)
										{
												wrist.setPosition(RobotParams.WRIST_BACKBOARD_POS);
										}
										else if (actionParams.pos < RobotParams.ARM_ROTATOR_LOAD_DEG)
										{
												wrist.setPosition(RobotParams.WRIST_COLLECTING_POS);
										}
										// Move arm to desired position
										rotator.setPosition(
														null, delay, actionParams.pos, true, actionParams.powerLimit, actionParams.completionEvent,
														timeout);
								}
								break;

						case RotatorSetPower:
								// Trace only if power has changed
								if (actionParams.power != armPrevPrintedPower)
								{
										tracer.traceInfo(moduleName, "actionParams=" + actionParams);
										armPrevPrintedPower = actionParams.power;
								}
								// Move the arm only if the elevator is already at or has reached safe height.
								// actionParams.actionEvent is null if elevator is already at safe height.
								if (actionParams.actionEvent == null || actionParams.actionEvent.isSignaled())
								{
										// Delay is only applicable for first movement. If elevator has moved, it already applied delay
										delay = actionParams.actionEvent == null ? actionParams.delay : 0.0;
										// Move arm with desired power
										rotator.setPower(null, delay, actionParams.power, actionParams.duration, actionParams.completionEvent);
								}
								break;

						case RotatorSetPidPower:
								// Trace only if power has changed
								if (actionParams.power != armPrevPrintedPower)
								{
										tracer.traceInfo(moduleName, "actionParams=" + actionParams);
										armPrevPrintedPower = actionParams.power;
								}
								// Move the arm only if the elevator is already at or has reached safe height.
								// actionParams.actionEvent is null if elevator is already at safe height.
								if (actionParams.actionEvent == null || actionParams.actionEvent.isSignaled())
								{
										// Move arm with desired power with PID control
										rotator.setPidPower(null, actionParams.power, actionParams.minPos, actionParams.maxPos, true);
								}
								break;
				}
				removePendingEventCallback(actionParams.actionEvent);
		} // performAction


		/**
		 * This method sets the rotator, extender and wrist to the loading position and makes sure it
		 * doesn't hit the intake on its way.
		 *
		 * @param owner specifies the owner ID to check if the caller has ownership of the subsystems.
		 * @param delay specifies the delay before moving in seconds.
		 * @param completionEvent specifies the event to signal when operation is completed, can be null if not provided.
		 * @param timeout specifies the maximum time allowed for the operation, can be zero if no timeout.
		 */
		public void setLoadingPosition(String owner, double delay, TrcEvent completionEvent, double timeout)
		{
				double expiredTime = timeout == 0.0 ? 0.0 : TrcTimer.getCurrentTime() + timeout;

				tracer.traceInfo(
								moduleName, "owner=%s, delay=%.3f, event=%s, timeout=%.3f (elevatorPos=%.2f, armPos=%.2f)",
								owner, delay, completionEvent, timeout, extender.getPosition(), rotator.getPosition());
				cancel(owner);
				TrcEvent releaseOwnershipEvent = acquireOwnership(owner, completionEvent, tracer);
				if (releaseOwnershipEvent != null) completionEvent = releaseOwnershipEvent;

				if (validateOwnership(owner))
				{
						// Move the wrist in loading position before lowering the elevator
						wrist.setPosition(RobotParams.WRIST_COLLECTING_POS);

						// Before rotating the arm, make sure the arm is safe to move (down to collecting pos,)
						// if not, we have to retract the arm to the safe position first before rotating the
						// arm back in. Then we will rotate the arm down.
						if (!rotatorIsSafeToMove(RobotParams.ARM_ROTATOR_LOAD_DEG))
						{
								// Retract arm to safe position first. Extender is fast and rotator is slow, so we
								// don't have to wait for the extender to complete its movement.
								extender.setPosition(
												null, 0.0, RobotParams.ARM_EXTENDER_SAFE_POS, true, RobotParams.ARM_EXTENDER_POWER_LIMIT, null, 0.0);
						}
						// Rotate the arm to loading position before retracting the arm @TODO check order of operations
						ActionParams actionParams = new ActionParams();
						TrcEvent actionEvent = new TrcEvent("setLoadingPosition.actionEvent");
						// Setting up the elevator operation after the arm is in loading position.
						actionParams.setPositionParams(
										ActionType.ExtenderSetPosition, actionEvent, 0.0, RobotParams.ARM_EXTENDER_LOAD_POS,
										RobotParams.ARM_EXTENDER_POWER_LIMIT, completionEvent, expiredTime);
						setActionCallback(actionParams, actionEvent);
						rotatorSetPosition(
										owner, false, delay, RobotParams.ARM_ROTATOR_LOAD_DEG, RobotParams.ARM_ROTATOR_POWER_LIMIT, actionEvent, 3.0);
				}
		} // setLoadingPosition

		/**
		 * This method sets the rotator, extender and wrist to the scoring position and makes sure it
		 * doesn't hit the intake on its way.
		 *
		 * @param owner specifies the owner ID to check if the caller has ownership of the subsystems.
		 * @param delay specifies the delay before moving in seconds.
		 * @param elevatorPos specifies the elevator scoring level.
		 * @param completionEvent specifies the event to signal when operation is completed, can be null if not provided.
		 * @param timeout specifies the maximum time allowed for the operation, can be zero if no timeout.
		 */
		public void setScoringPosition(
						String owner, double delay, double elevatorPos, TrcEvent completionEvent, double timeout)
		{
				// @TODO Implement Scoring
//				double expiredTime = timeout == 0.0 ? 0.0 : TrcTimer.getCurrentTime() + timeout;
//
//				tracer.traceInfo(
//								moduleName,
//								"owner=%s, delay=%.3f, elevatorPos=%.1f, event=%s, timeout=%.3f (elevatorPos=%.2f, armPos=%.2f)",
//								owner, delay, elevatorPos, completionEvent, timeout, extender.getPosition(), rotator.getPosition());
//				cancel(owner);
//				TrcEvent releaseOwnershipEvent = acquireOwnership(owner, completionEvent, tracer);
//				if (releaseOwnershipEvent != null) completionEvent = releaseOwnershipEvent;
//
//				if (validateOwnership(owner))
//				{
//						ActionParams actionParams = new ActionParams();
//						TrcEvent actionEvent = new TrcEvent("setScoringPosition.actionEvent");
//						// Setting up the arm operation after the elevator is in scoring height.
//						actionParams.setPositionParams(
//										ActionType.RotatorSetPosition, actionEvent, 0.0, RobotParams.ARM_ROTATOR_SCORE_BACKDROP_POS,
//										RobotParams.ARM_ROTATOR_POWER_LIMIT, completionEvent, expiredTime);
//						// Wrist position will be set in the arm operation.
//						// Move the elevator to scoring height before moving the arm to scoring position.
//						setActionCallback(actionParams, actionEvent);
//						elevatorSetPosition(owner, false, delay, elevatorPos, RobotParams.ARM_EXTENDER_POWER_LIMIT, actionEvent, 1.0);
//				}
		} // setScoringPosition

		/**
		 * This method sets the elevator and arm to the hanging position and makes sure it doesn't hit the intake on its
		 * way.
		 *
		 * @param owner specifies the owner ID to check if the caller has ownership of the subsystems.
		 * @param delay specifies the delay before moving in seconds.
		 * @param completionEvent specifies the event to signal when operation is completed, can be null if not provided.
		 * @param timeout specifies the maximum time allowed for the operation, can be zero if no timeout.
		 */
		public void setHangingPosition(String owner, double delay, TrcEvent completionEvent, double timeout)
		{
				// @TODO Implement hanging
//				double expiredTime = timeout == 0.0 ? 0.0 : TrcTimer.getCurrentTime() + timeout;
//
//				tracer.traceInfo(
//								moduleName, "owner=%s, delay=%.3f, event=%s, timeout=%.3f", owner, delay, completionEvent, timeout);
//				cancel(owner);
//				TrcEvent releaseOwnershipEvent = acquireOwnership(owner, completionEvent, tracer);
//				if (releaseOwnershipEvent != null) completionEvent = releaseOwnershipEvent;
//
//				if (validateOwnership(owner))
//				{
//						ActionParams actionParams = new ActionParams();
//						TrcEvent actionEvent = new TrcEvent("setHangingPosition.actionEvent");
//						// Setting up the arm operation after the elevator is in max height.
//						actionParams.setPositionParams(
//										ActionType.RotatorSetPosition, actionEvent, 0.0, RobotParams.ARM_ROTATOR_HANG_POS, 0.1,
//										completionEvent, expiredTime);
//						// Move the elevator to max height before moving the arm to hanging position.
//						setActionCallback(actionParams, actionEvent);
//						elevatorSetPosition(
//										owner, false, delay, RobotParams.ARM_EXTENDER_MAX_POS - 0.5, RobotParams.ARM_EXTENDER_POWER_LIMIT, actionEvent,
//										1.0);
//				}
		} // setHangingPosition


		// ------------------------------------------------------------------------------------------ //
		// -------------------------------- Rotator subsystem methods ------------------------------- //
		// ------------------------------------------------------------------------------------------ //
//		/** @TODO Check if arm rotator needs gravity compensation (if not worm gear)
//		 * This method is called to compute the power compensation to counteract gravity on the Arm
//		 * (rotator).
//		 *
//		 * @param currPower specifies the current motor power (not used).
//		 * @return gravity compensation for the arm rotator.
//		 */
//		private double armRotatorGetPowerComp(double currPower)
//		{
//				return RobotParams.ARM_ROTATOR_MAX_POWER_COMP * -Math.cos(Math.toRadians(rotator.getPosition()));
//		} // armGetPowerComp

		/**
		 * This method checks if the arm is safe to rotate so it doesn't hit the frame of the robot.
		 *
		 * @param rotatorTargetPos specifies the arm rotator target position.
		 * @return true if it is safe to move, false otherwise.
		 */
		private boolean rotatorIsSafeToMove(double rotatorTargetPos)
		{
				double rotatorPos = rotator.getPosition();

//				Want to move from load to scoring
//				Arm can only move if: extension is within safe pos and wrist is down.
//
//						want to move from scoring to loading
//				Arm can only move if: extension is within safe pos and wrist is down
				

//				return extender.getPosition() + RobotParams.ARM_EXTENDER_TOLERANCE <= RobotParams.ARM_EXTENDER_SAFE_POS ||
//								// ^^^ Extender is retracted in the robot
//								rotatorPos <= RobotParams.ARM_ROTATOR_LOAD_DEG && rotatorTargetPos <= RobotParams.ARM_ROTATOR_LOAD_DEG ||
//								// ^^^ Rotator already in LOAD_POS
//								rotatorPos + RobotParams.ARM_ROTATOR_TOLERANCE >= RobotParams.ARM_ROTATOR_SAFE_DEG &&
//												rotatorTargetPos >= RobotParams.ARM_ROTATOR_SAFE_DEG;
		} // rotatorIsSafeToMove

		/**
		 * This method sets the arm position.
		 *
		 * @param owner specifies the ID string of the caller for checking ownership, can be null if caller is not
		 *        ownership aware.
		 * @param cancelPrev specifies true to cancel previous operation, false otherwise.
		 * @param delay specifies the time in seconds to delay before setting the value, 0.0 if no delay.
		 * @param pos specifies the position in scaled units to be set.
		 * @param powerLimit specifies the maximum power output limits.
		 * @param completionEvent specifies the event to signal when operation is completed, can be null if not provided.
		 * @param timeout specifies timeout in seconds.
		 */
		private void rotatorSetPosition(
						String owner, boolean cancelPrev, double delay, double pos, double powerLimit, TrcEvent completionEvent,
						double timeout)
		{
				double expiredTime = timeout == 0.0 ? 0.0 : TrcTimer.getCurrentTime() + timeout;

				tracer.traceInfo(
								moduleName,
								"owner=%s, delay=%.3f, pos=%.1f, powerLimit=%.1f, event=%s, timeout=%.3f " +
												"(elevatorPos=%.2f, armPos=%.2f)",
								owner, delay, pos, powerLimit, completionEvent, timeout, extender.getPosition(), rotator.getPosition());
				if (cancelPrev)
				{
						cancel(owner);
				}
				TrcEvent releaseOwnershipEvent = acquireOwnership(owner, completionEvent, tracer);
				if (releaseOwnershipEvent != null) completionEvent = releaseOwnershipEvent;

				if (validateOwnership(owner))
				{
						boolean safeToMove = rotatorIsSafeToMove(pos);
						ActionParams actionParams = new ActionParams();
						TrcEvent actionEvent = safeToMove ? null : new TrcEvent("armSetPosition.actionEvent");
						// Setting up the arm operation after the elevator is at safe height.
						actionParams.setPositionParams(
										ActionType.RotatorSetPosition, actionEvent, delay, pos, powerLimit, completionEvent, expiredTime);
						if (safeToMove)
						{
								// Elevator is already at safe height.
								performAction(actionParams);
						}
						else
						{
								// Move the elevator to safe height and set up a callback to finish the arm operation.
								setActionCallback(actionParams, actionEvent);
								extender.setPosition(
												null, delay, RobotParams.ARM_EXTENDER_SAFE_POS, true, RobotParams.ARM_EXTENDER_POWER_LIMIT, actionEvent,
												timeout);
						}
				}
		}   //armSetPosition

		/**
		 * This method sets the arm position.
		 *
		 * @param owner specifies the ID string of the caller for checking ownership, can be null if caller is not
		 *        ownership aware.
		 * @param delay specifies the time in seconds to delay before setting the value, 0.0 if no delay.
		 * @param pos specifies the position in scaled units to be set.
		 * @param powerLimit specifies the maximum power output limits.
		 * @param completionEvent specifies the event to signal when operation is completed, can be null if not provided.
		 * @param timeout specifies timeout in seconds.
		 */
		public void rotatorSetPosition(
						String owner, double delay, double pos, double powerLimit, TrcEvent completionEvent, double timeout)
		{
				rotatorSetPosition(owner, true, delay, pos, powerLimit, completionEvent, timeout);
		}   //armSetPosition

		/**
		 * This method sets the arm power.
		 *
		 * @param owner specifies the ID string of the caller for checking ownership, can be null if caller is not
		 *        ownership aware.
		 * @param delay specifies the time in seconds to delay before setting the value, 0.0 if no delay.
		 * @param power specifies the percentage power (range -1.0 to 1.0).
		 * @param duration specifies the duration in seconds to run the motor and turns it off afterwards, 0.0 if not
		 *        turning off.
		 * @param event specifies the event to signal when the motor operation is completed.
		 */
		public void armSetPower(String owner, double delay, double power, double duration, TrcEvent event)
		{
				// Trace only if power has changed.
				if (power != armPrevPrintedPower)
				{
						tracer.traceInfo(
										moduleName, "owner=%s, delay=%.1f, power=%.3f, duration=%.3f, pos=%.3f, event=%s, safe=%s",
										owner, delay, power, duration, rotator.getPosition(), event,
										rotatorIsSafeToMove(power > 0.0 ? RobotParams.ARM_ROTATOR_MAX_DEG : RobotParams.ARM_ROTATOR_MIN_DEG));
						armPrevPrintedPower = power;
				}

				if (validateOwnership(owner))
				{
						boolean safeToMove =
										power == 0.0 || rotatorIsSafeToMove(power > 0.0 ? RobotParams.ARM_ROTATOR_MAX_DEG : RobotParams.ARM_ROTATOR_MIN_DEG);
						ActionParams actionParams = new ActionParams();
						TrcEvent actionEvent = safeToMove ? null : new TrcEvent("armSetPower.actionEvent");
						// Setting up the arm operation after the elevator is at safe height.
						actionParams.setPowerParams(ActionType.RotatorSetPower, actionEvent, delay, power, duration, event);
						if (safeToMove)
						{
								// Elevator is already at safe height.
								performAction(actionParams);
						}
						else
						{
								// Move the elevator to safe height and set up a callback to finish the arm operation.
								setActionCallback(actionParams, actionEvent);
								extender.setPosition(
												null, delay, RobotParams.ARM_EXTENDER_SAFE_POS, true, RobotParams.ARM_EXTENDER_POWER_LIMIT, actionEvent,
												0.0);
						}
				}
		}   //armSetPower

		/**
		 * This method sets the arm power with PID control.
		 *
		 * @param owner specifies the owner ID to check if the caller has ownership of the motor.
		 * @param power specifies the upper bound power of the motor.
		 * @param minPos specifies the minimum of the position range.
		 * @param maxPos specifies the maximum of the position range.
		 */
		public void armSetPidPower(String owner, double power, double minPos, double maxPos)
		{
				// Trace only if power has changed.
				if (power != armPrevPrintedPower)
				{
						tracer.traceInfo(
										moduleName, "owner=%s, power=%.3f, pos=%.3f, minPos=%.1f, maxPos=%.1f, safe=%s",
										owner, power, rotator.getPosition(), minPos, maxPos, rotatorIsSafeToMove(power > 0.0 ? maxPos : minPos));
						armPrevPrintedPower = power;
				}

				if (validateOwnership(owner))
				{
						boolean safeToMove =
										power == 0.0 || rotatorIsSafeToMove(power > 0.0 ? maxPos : minPos);
						ActionParams actionParams = new ActionParams();
						TrcEvent actionEvent = safeToMove ? null : new TrcEvent("armSetPidPower.actionEvent");
						// Setting up the arm operation after the elevator is at safe height.
						actionParams.setPidPowerParams(ActionType.RotatorSetPidPower, actionEvent, power, minPos, maxPos);
						if (safeToMove)
						{
								// Elevator is already at safe height.
								performAction(actionParams);
						}
						else
						{
								// Move the elevator to safe height and set up a callback to finish the arm operation.
								setActionCallback(actionParams, actionEvent);
								extender.setPosition(
												null, 0.0, RobotParams.ARM_EXTENDER_SAFE_POS, true, RobotParams.ARM_EXTENDER_POWER_LIMIT, actionEvent, 0.0);
						}
				}
		}   //armSetPidPower

		/**
		 * This method sets the arm to the specified preset position.
		 *
		 * @param owner specifies the owner ID to check if the caller has ownership of the subsystem.
		 * @param delay specifies delay time in seconds before setting position, can be zero if no delay.
		 * @param presetIndex specifies the index to the preset position array.
		 * @param powerLimit specifies the power limit applied to the elevator.
		 * @param event specifies the event to signal when done, can be null if not provided.
		 * @param timeout specifies a timeout value in seconds. If the operation is not completed without the specified
		 *        timeout, the operation will be canceled and the event will be signaled. If no timeout is specified, it
		 *        should be set to zero.
		 */
		public void armSetPresetPosition(
						String owner, double delay, int presetIndex, double powerLimit, TrcEvent event, double timeout)
		{
				if (rotator.validatePresetIndex(presetIndex))
				{
						rotatorSetPosition(owner, delay, rotator.getPresetValue(presetIndex), powerLimit, event, timeout);
				}
		}   //armSetPresetPosition

		/**
		 * This method sets the arm to the next preset position up from the current position.
		 *
		 * @param owner specifies the owner ID that will acquire ownership before setting the preset position and will
		 *        automatically release ownership when the actuator movement is completed, can be null if no ownership
		 *        is required.
		 * @param powerLimit specifies the power limit applied to the elevator.
		 */
		public void armPresetPositionUp(String owner, double powerLimit)
		{
				int index = rotator.nextPresetIndexUp();

				if (index != -1)
				{
						rotatorSetPosition(owner, 0.0, rotator.getPresetValue(index), powerLimit, null, 5.0);
				}
		}   //armPresetPositionUp

		/**
		 * This method sets the arm to the next preset position down from the current position.
		 *
		 * @param owner specifies the owner ID that will acquire ownership before setting the preset position and will
		 *        automatically release ownership when the actuator movement is completed, can be null if no ownership
		 *        is required.
		 * @param powerLimit specifies the power limit applied to the elevator.
		 */
		public void armPresetPositionDown(String owner, double powerLimit)
		{
				int index = rotator.nextPresetIndexDown();

				if (index != -1)
				{
						rotatorSetPosition(owner, 0.0, rotator.getPresetValue(index), powerLimit, null, 5.0);
				}
		}   //armPresetPositionDown


		// ------------------------------------------------------------------------------------------ //
		// -------------------------------- Extender subsystem methods ------------------------------ //
		// ------------------------------------------------------------------------------------------ //
		/**
		 * This method checks if the elevator is safe to move to the target position so it doesn't hit the intake.
		 *
		 * @param extenderTargetPos specifies the elevator target position.
		 * @return true if it is safe to move, false otherwise.
		 */
		private boolean extenderIsSafeToMove(double extenderTargetPos)
		{
				double currArmPos = rotator.getPosition();

				return currArmPos - RobotParams.ARM_ROTATOR_TOLERANCE <= RobotParams.ARM_ROTATOR_LOAD_DEG ||
								// ^^^ Arm is already in load position.
								currArmPos + RobotParams.ARM_ROTATOR_TOLERANCE >= RobotParams.ARM_ROTATOR_FREE_TO_MOVE_POS ||
								// ^^^ Arm is already beyond FREE_TO_MOVE position.
								extenderTargetPos >= RobotParams.ARM_EXTENDER_SAFE_POS;
				// ^^^ We are moving up beyond ELEVATOR_SAFE_POS.
		}   //elevatorIsSafeToMove

		/**
		 * This method sets the elevator position.
		 *
		 * @param owner specifies the ID string of the caller for checking ownership, can be null if caller is not
		 *        ownership aware.
		 * @param cancelPrev specifies true to cancel previous operation, false otherwise.
		 * @param delay specifies the time in seconds to delay before setting the value, 0.0 if no delay.
		 * @param pos specifies the position in scaled units to be set.
		 * @param powerLimit specifies the maximum power output limits.
		 * @param completionEvent specifies the event to signal when operation is completed, can be null if not provided.
		 * @param timeout specifies timeout in seconds.
		 */
		private void elevatorSetPosition(
						String owner, boolean cancelPrev, double delay, double pos, double powerLimit, TrcEvent completionEvent,
						double timeout)
		{
				double expiredTime = timeout == 0.0 ? 0.0 : TrcTimer.getCurrentTime() + timeout;

				tracer.traceInfo(
								moduleName,
								"owner=%s, delay=%.3f, pos=%.1f, powerLimit=%.1f, event=%s, timeout=%.3f " +
												"(elevatorPos=%.2f, armPos=%.2f)",
								owner, delay, pos, powerLimit, completionEvent, timeout, extender.getPosition(), rotator.getPosition());
				if (cancelPrev)
				{
						cancel(owner);
				}
				TrcEvent releaseOwnershipEvent = acquireOwnership(owner, completionEvent, tracer);
				if (releaseOwnershipEvent != null) completionEvent = releaseOwnershipEvent;

				if (validateOwnership(owner))
				{
						boolean safeToMove = extenderIsSafeToMove(pos);
						ActionParams actionParams = new ActionParams();
						TrcEvent actionEvent = safeToMove ? null : new TrcEvent("elevatorSetPosition.actionEvent");
						// Setting up the elevator operation after the arm is at safe position.
						actionParams.setPositionParams(
										ActionType.ExtenderSetPosition, actionEvent, delay, pos, powerLimit, completionEvent, expiredTime);
						if (safeToMove)
						{
								// Arm is already at safe position.
								performAction(actionParams);
						}
						else
						{
								// Move the arm to safe position and set up a callback to finish the elevator operation.
								setActionCallback(actionParams, actionEvent);
								rotator.setPosition(
												null, delay, RobotParams.ARM_ROTATOR_LOAD_DEG, true, RobotParams.ARM_ROTATOR_POWER_LIMIT, actionEvent, timeout);
						}
				}
		}   //elevatorSetPosition

		/**
		 * This method sets the elevator position.
		 *
		 * @param owner specifies the ID string of the caller for checking ownership, can be null if caller is not
		 *        ownership aware.
		 * @param delay specifies the time in seconds to delay before setting the value, 0.0 if no delay.
		 * @param pos specifies the position in scaled units to be set.
		 * @param powerLimit specifies the maximum power output limits.
		 * @param completionEvent specifies the event to signal when operation is completed, can be null if not provided.
		 * @param timeout specifies timeout in seconds.
		 */
		public void elevatorSetPosition(
						String owner, double delay, double pos, double powerLimit, TrcEvent completionEvent, double timeout)
		{
				elevatorSetPosition(owner, true, delay, pos, powerLimit, completionEvent, timeout);
		}   //elevatorSetPosition

		/**
		 * This method sets the elevator power.
		 *
		 * @param owner specifies the ID string of the caller for checking ownership, can be null if caller is not
		 *        ownership aware.
		 * @param delay specifies the time in seconds to delay before setting the value, 0.0 if no delay.
		 * @param power specifies the percentage power (range -1.0 to 1.0).
		 * @param duration specifies the duration in seconds to run the motor and turns it off afterwards, 0.0 if not
		 *        turning off.
		 * @param event specifies the event to signal when the motor operation is completed.
		 */
		public void elevatorSetPower(String owner, double delay, double power, double duration, TrcEvent event)
		{
				// Trace only if power has changed.
				if (power != elevatorPrevPrintedPower)
				{
						tracer.traceInfo(
										moduleName, "owner=%s, delay=%.1f, power=%.3f, duration=%.3f, pos=%.3f, event=%s, safe=%s",
										owner, delay, power, duration, extender.getPosition(), event,
										extenderIsSafeToMove(power > 0.0 ? RobotParams.ARM_EXTENDER_MAX_POS : RobotParams.ARM_EXTENDER_MIN_POS));
						elevatorPrevPrintedPower = power;
				}

				if (validateOwnership(owner))
				{
						boolean safeToMove =
										power == 0.0 ||
														extenderIsSafeToMove(power > 0.0 ? RobotParams.ARM_EXTENDER_MAX_POS : RobotParams.ARM_EXTENDER_MIN_POS);
						ActionParams actionParams = new ActionParams();
						TrcEvent actionEvent = safeToMove ? null : new TrcEvent("elevatorSetPower.actionEvent");
						// Setting up the elevator operation after the arm is at safe position.
						actionParams.setPowerParams(ActionType.ExtenderSetPower, actionEvent, delay, power, duration, event);
						if (safeToMove)
						{
								// Arm is already at safe position.
								performAction(actionParams);
						}
						else
						{
								// Move the arm to safe position and set up a callback to finish the elevator operation.
								setActionCallback(actionParams, actionEvent);
								rotator.setPosition(
												null, delay, RobotParams.ARM_ROTATOR_LOAD_DEG, true, RobotParams.ARM_ROTATOR_POWER_LIMIT, actionEvent, 0.0);
						}
				}
		}   //elevatorSetPower

		/**
		 * This method sets the elevator power with PID control.
		 *
		 * @param owner specifies the owner ID to check if the caller has ownership of the motor.
		 * @param power specifies the upper bound power of the motor.
		 * @param minPos specifies the minimum of the position range.
		 * @param maxPos specifies the maximum of the position range.
		 */
		public void elevatorSetPidPower(String owner, double power, double minPos, double maxPos)
		{
				// Trace only if power has changed.
				if (power != elevatorPrevPrintedPower)
				{
						tracer.traceInfo(
										moduleName, "owner=%s, power=%.3f, pos=%.3f, minPos=%.1f, maxPos=%.1f, safe=%s",
										owner, power, extender.getPosition(), minPos, maxPos,
										extenderIsSafeToMove(power > 0.0 ? maxPos : minPos));
						elevatorPrevPrintedPower = power;
				}

				if (validateOwnership(owner))
				{
						boolean safeToMove = power == 0.0 || extenderIsSafeToMove(power > 0.0 ? maxPos : minPos);
						ActionParams actionParams = new ActionParams();
						TrcEvent actionEvent = safeToMove ? null : new TrcEvent("elevatorSetPidPower.actionEvent");
						// Setting up the elevator operation after the arm is at safe position.
						actionParams.setPidPowerParams(ActionType.ExtenderSetPidPower, actionEvent, power, minPos, maxPos);
						if (safeToMove)
						{
								// Arm is already at safe position.
								performAction(actionParams);
						}
						else
						{
								// Move the arm to safe position and set up a callback to finish the elevator operation.
								setActionCallback(actionParams, actionEvent);
								rotator.setPosition(
												null, 0.0, RobotParams.ARM_ROTATOR_LOAD_DEG, true, RobotParams.ARM_ROTATOR_POWER_LIMIT, actionEvent, 0.0);
						}
				}
		}   //elevatorSetPidPower

		/**
		 * This method sets the elevator to the specified preset position.
		 *
		 * @param owner specifies the owner ID to check if the caller has ownership of the subsystem.
		 * @param delay specifies delay time in seconds before setting position, can be zero if no delay.
		 * @param presetIndex specifies the index to the preset position array.
		 * @param powerLimit specifies the power limit applied to the elevator.
		 * @param event specifies the event to signal when done, can be null if not provided.
		 * @param timeout specifies a timeout value in seconds. If the operation is not completed without the specified
		 *        timeout, the operation will be canceled and the event will be signaled. If no timeout is specified, it
		 *        should be set to zero.
		 */
		public void elevatorSetPresetPosition(
						String owner, double delay, int presetIndex, double powerLimit, TrcEvent event, double timeout)
		{
				if (extender.validatePresetIndex(presetIndex))
				{
						elevatorSetPosition(owner, delay, extender.getPresetValue(presetIndex), powerLimit, event, timeout);
				}
		}   //elevatorSetPresetPosition

		/**
		 * This method sets the elevator to the next preset position up from the current position.
		 *
		 * @param owner specifies the owner ID that will acquire ownership before setting the preset position and will
		 *        automatically release ownership when the actuator movement is completed, can be null if no ownership
		 *        is required.
		 * @param powerLimit specifies the power limit applied to the elevator.
		 */
		public void elevatorPresetPositionUp(String owner, double powerLimit)
		{
				int index = extender.nextPresetIndexUp();

				if (index != -1)
				{
						elevatorSetPosition(owner, 0.0, extender.getPresetValue(index), powerLimit, null, 1.0);
				}
		}   //elevatorPresetPositionUp

		/**
		 * This method sets the elevator to the next preset position down from the current position.
		 *
		 * @param owner specifies the owner ID that will acquire ownership before setting the preset position and will
		 *        automatically release ownership when the actuator movement is completed, can be null if no ownership
		 *        is required.
		 * @param powerLimit specifies the power limit applied to the elevator.
		 */
		public void elevatorPresetPositionDown(String owner, double powerLimit)
		{
				int index = extender.nextPresetIndexDown();

				if (index != -1)
				{
						elevatorSetPosition(owner, 0.0, extender.getPresetValue(index), powerLimit, null, 1.0);
				}
		}   //elevatorPresetPositionDown

		/**
		 * This method sets the wrist position while observing whether it's safe to do so.
		 *
		 * @param owner specifies the owner ID to check if the caller has ownership of the subsystem.
		 * @param delay specifies the delay in seconds before setting the position of the servo, can be zero if no delay.
		 * @param position specifies the physical position of the servo motor. This value may be in degrees if
		 *        setPhysicalPosRange is called with the degree range.
		 * @param completionEvent specifies an event object to signal when the timeout event has expired.
		 * @param timeout specifies a maximum time value the operation should be completed in seconds.
		 */
		public void wristSetPosition(String owner, double delay, double position, TrcEvent completionEvent, double timeout)
		{
				if (extender.getPosition() >= RobotParams.ARM_EXTENDER_SAFE_POS ||
								rotator.getPosition() >= RobotParams.ARM_ROTATOR_SAFE_DEG)
				{
						wrist.setPosition(owner, delay, position, completionEvent, timeout);
				}
		}   //wristSetPosition

		/**
		 * This method sets the wrist position while observing whether it's safe to do so.
		 *
		 * @param delay specifies the delay in seconds before setting the position of the servo, can be zero if no delay.
		 * @param position specifies the physical position of the servo motor. This value may be in degrees if
		 *        setPhysicalPosRange is called with the degree range.
		 * @param completionEvent specifies an event object to signal when the timeout event has expired.
		 * @param timeout specifies a maximum time value the operation should be completed in seconds.
		 */
		public void wristSetPosition(double delay, double position, TrcEvent completionEvent, double timeout)
		{
				wristSetPosition(null, delay, position, completionEvent, timeout);
		}   //wristSetPosition

		/**
		 * This method sets the wrist position while observing whether it's safe to do so.
		 *
		 * @param delay specifies the delay in seconds before setting the position of the servo, can be zero if no delay.
		 * @param position specifies the physical position of the servo motor. This value may be in degrees if
		 *        setPhysicalPosRange is called with the degree range.
		 */
		public void wristSetPosition(double delay, double position)
		{
				wristSetPosition(null, delay, position, null, 0.0);
		}   //wristSetPosition

		/**
		 * This method sets the wrist position while observing whether it's safe to do so.
		 *
		 * @param position specifies the physical position of the servo motor. This value may be in degrees if
		 *        setPhysicalPosRange is called with the degree range.
		 */
		public void wristSetPosition(double position)
		{
				wristSetPosition(null, 0.0, position, null, 0.0);
		}   //wristSetPosition

		/**
		 * This method sets the Wrist Sensor task to be enabled/ disabled. If enabled, it is enabled
		 * as a post periodic task.
		 *
		 * @param enabled true to enable, false to disable.
		 */
		private void wristSetSensorTaskEnabled(boolean enabled)
		{
				boolean taskIsEnabled = wristSensorTaskObj.isRegistered();

				if (!taskIsEnabled && enabled)
				{
						wristSensorTaskObj.registerTask(TrcTaskMgr.TaskType.POST_PERIODIC_TASK);
				}
				else if (taskIsEnabled && !enabled)
				{
						wristSensorTaskObj.unregisterTask();
				}
		}

		/**
		 * This method cancels the wrist sensor alignment task.
		 */
		private void wristCancelSensorTask()
		{
				wristSetSensorTaskEnabled(false);
				
				if (wristCompletionEvent != null)
				{
						wristCompletionEvent.cancel();
				}
		}

		/**
		 * This method enabled the Wrist Sensor Task to align the wrist to the backdrop, and signals
		 * the event when the alignment is complete. Doesn't do anything if wrist sensors do not exist.
		 * 
		 * @param event Event to trigger when completed.
		 */
		public void wristAlignment(TrcEvent event)
		{
				if (wristSensorTop != null && wristSensorBot != null)
				{
						this.wristCompletionEvent = event;

						wristSetSensorTaskEnabled(true);
				}
		}

		/**
		 * This task gets the error from the wrist sensor and
		 *
		 * @param taskType
		 * @param runMode
		 * @param slowPeriodicLoop
		 */
		private void wristSensorTask(TrcTaskMgr.TaskType taskType, TrcRobot.RunMode runMode, boolean slowPeriodicLoop)
		{
				double wristError = wristGetDistance(wristSensorTop) - wristGetDistance(wristSensorBot);
				
				if (Math.abs(wristError) <= RobotParams.WRIST_TOLERANCE)
				{
						wristSetSensorTaskEnabled(false);
						
						if (wristCompletionEvent != null)
						{
								wristCompletionEvent.signal();
						}
				}
				else
				{
						wrist.setPower(RobotParams.WRIST_KP * wristError);
				}
		}

		/**
		 * This method is called the TrcTriggerThresholdZones to get the sensor data.
		 *
		 * @return distance to detected object in inches. Returns NaN if sensor does not exist.
		 */
		public double wristGetDistance(FtcDistanceSensor distanceSensor)
		{
				if (distanceSensor != null)
				{
						TrcSensor.SensorData<Double> data =
										distanceSensor.getProcessedData(0, FtcDistanceSensor.DataType.DISTANCE_INCH);
						return data != null && data.value != null ? data.value : 0.0;
				}

				return Double.NaN;
		} // wristGetDistance

} // class ElevatorArm