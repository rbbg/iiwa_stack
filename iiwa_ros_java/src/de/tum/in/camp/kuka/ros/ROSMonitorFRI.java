package de.tum.in.camp.kuka.ros;


import java.net.URI;

import org.ros.address.BindAddress;
import org.ros.node.DefaultNodeMainExecutor;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMainExecutor;
import org.ros.time.NtpTimeProvider;

import com.kuka.connectivity.fri.FRIChannelInformation;
import com.kuka.connectivity.fri.FRIConfiguration;
import com.kuka.connectivity.fri.FRISession;
import com.kuka.connectivity.fri.IFRISessionListener;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.geometricModel.Tool;
import com.kuka.roboticsAPI.motionModel.SmartServo;
import com.kuka.roboticsAPI.motionModel.controlModeModel.JointImpedanceControlMode;
import com.kuka.roboticsAPI.uiModel.userKeys.IUserKey;
import com.kuka.roboticsAPI.uiModel.userKeys.IUserKeyBar;
import com.kuka.roboticsAPI.uiModel.userKeys.IUserKeyListener;
import com.kuka.roboticsAPI.uiModel.userKeys.UserKeyAlignment;
import com.kuka.roboticsAPI.uiModel.userKeys.UserKeyEvent;

public class ROSMonitorFRI extends RoboticsAPIApplication {
	private LBR robot;
	private Tool tool;
	private SmartServo motion;
	
	private boolean initSuccessful = false;
	private boolean debug = false;
	
	private FRISession session;
	private iiwaConfiguration configuration; //< Configuration via parameters and services.

	// ROS Configuration and Node execution objects.
	private NodeConfiguration nodeConfConfiguration;
	private NodeMainExecutor nodeExecutor;
	
	// gravity compensation stuff
	private IUserKeyBar gravcompKeybar;
	private IUserKey gravCompKey;
	private IUserKeyListener gravCompKeyList;
	private boolean gravCompEnabled = false;
	private boolean gravCompSwitched = false;
	
	
	
	public void initialize() {
		robot = getContext().getDeviceFromType(LBR.class);		
		// standard stuff
		configuration = new iiwaConfiguration();
		
		// gravity compensation - only in ROSMonitor for safety
		gravcompKeybar = getApplicationUI().createUserKeyBar("Gravcomp");
		gravCompKeyList = new IUserKeyListener() {
			@Override
			public void onKeyEvent(IUserKey key, com.kuka.roboticsAPI.uiModel.userKeys.UserKeyEvent event) {
				if (event == UserKeyEvent.FirstKeyDown) {
					gravCompEnabled = true;
					gravCompSwitched = true;
				} else if (event == UserKeyEvent.SecondKeyDown) {
					gravCompEnabled = false;
					gravCompSwitched = true;
				}
			}
		};
		gravCompKey = gravcompKeybar.addDoubleUserKey(0, gravCompKeyList, true);
		gravCompKey.setText(UserKeyAlignment.TopMiddle, "ON");
		gravCompKey.setText(UserKeyAlignment.BottomMiddle, "OFF");
		gravcompKeybar.publish();
	
		// ROS initialization
		try {
			URI uri = new URI(iiwaConfiguration.getMasterURI());
			nodeConfConfiguration = NodeConfiguration.newPublic(iiwaConfiguration.getRobotIp(), uri);
			nodeConfConfiguration.setTimeProvider(iiwaConfiguration.getTimeProvider());
			nodeConfConfiguration.setNodeName(iiwaConfiguration.getRobotName() + "/iiwa_configuration");
			nodeConfConfiguration.setTcpRosBindAddress(BindAddress.newPublic(30010));			
		}
		catch (Exception e) {
			if (debug) getLogger().info("Node Configuration failed; please check the ROS master IP in the Sunrise app source code");
			getLogger().info(e.toString());
			return;
		}

		try {
			// Start the Publisher node with the set up configuration.
			nodeExecutor = DefaultNodeMainExecutor.newDefault();
			nodeExecutor.execute(configuration, nodeConfConfiguration);
			if (debug) 
				getLogger().info("ROS Node initialized.");
		}
		catch(Exception e) {
			if (debug) 
				getLogger().info("Node Executor failed.");
			
			getLogger().info(e.toString());
			return;
		}
		
		FRIConfiguration FRIConf = FRIConfiguration.createRemoteConfiguration(robot, iiwaConfiguration.getMasterURI());
		FRIConf.setSendPeriodMilliSec(10); // TODO parametrize this
		session = new FRISession(FRIConf);
		getLogger().info("FRI Session was created.");

		// TODO : make something usefull out of it
		IFRISessionListener listener = new IFRISessionListener(){
				@Override 
				public void onFRIConnectionQualityChanged(FRIChannelInformation friChannelInformation){
				      getLogger().info("QualityChangedEvent - quality:" + friChannelInformation.getQuality());
				   }
				@Override public void onFRISessionStateChanged(FRIChannelInformation friChannelInformation){
				      getLogger().info("SessionStateChangedEvent - session state:" + friChannelInformation.getFRISessionState());
				   }
				};
		session.addFRISessionListener(listener);
			
		initSuccessful = true;  // we cannot throw here
}

	public void run() {
		if (!initSuccessful) {
			throw new RuntimeException("Could not init the RoboticApplication successfully");
		}
		
		try {
			configuration.waitForInitialization();
		} catch (InterruptedException e1) {
			e1.printStackTrace();
			return;
		}
		
		getLogger().info("using time provider: " + iiwaConfiguration.getTimeProvider().getClass().getSimpleName());

		motion = new SmartServo(robot.getCurrentJointPosition());
		motion.setMinimumTrajectoryExecutionTime(8e-3);
		motion.setJointVelocityRel(configuration.getDefaultRelativeJointSpeed());
		motion.setTimeoutAfterGoalReach(300);
				
		// Tool to attach
		String toolFromConfig = configuration.getToolName();
		if (toolFromConfig != "") {
			getLogger().info("attaching tool " + toolFromConfig);
			tool = (Tool)getApplicationData().createFromTemplate(toolFromConfig);
			tool.attachTo(robot.getFlange());
		} else {
			getLogger().info("no tool attached");
		}
				
		if (!SmartServo.validateForImpedanceMode(robot))
			getLogger().error("Too much external torque on the robot! Is it a singular position?");
		
		JointImpedanceControlMode controlMode = new JointImpedanceControlMode(7); // TODO!!
		robot.moveAsync(motion.setMode(controlMode));
		
		// The run loop
		getLogger().info("Starting the ROS Monitor loop...");
		try {
			while(true) { 

				if (iiwaConfiguration.getTimeProvider() instanceof org.ros.time.NtpTimeProvider) {
					((NtpTimeProvider) iiwaConfiguration.getTimeProvider()).updateTime();
				}
				
				
				if (gravCompEnabled) {
					if (gravCompSwitched) {
						gravCompSwitched = false;
						getLogger().warn("Enabling gravity compensation");
						controlMode.setStiffnessForAllJoints(0);
						controlMode.setDampingForAllJoints(0.7);
						motion.getRuntime().changeControlModeSettings(controlMode);
					}
					
					motion.getRuntime().setDestination(robot.getCurrentJointPosition());
				} else {
					if (gravCompSwitched) {
						gravCompSwitched = false;
						getLogger().warn("Disabling gravity compensation");
						controlMode.setStiffnessForAllJoints(1500);
						motion.getRuntime().changeControlModeSettings(controlMode);
						motion.getRuntime().setDestination(robot.getCurrentJointPosition());
					}
				}
			} 
		}
		catch (Exception e) {
			getLogger().info("ROS loop aborted. " + e.toString());
		} finally {
			if (nodeExecutor != null) {
				nodeExecutor.shutdownNodeMain(configuration);
				if (debug)getLogger().info("ROS Node terminated.");
			}
			session.close();
			getLogger().info("FRI Session was closed.");
			getLogger().info("ROS loop has ended. Application terminated.");
		}
	}

	@Override
	public void dispose() {
		// The Publisher node is killed.
		if (nodeExecutor != null) {
			nodeExecutor.shutdownNodeMain(configuration);
			getLogger().info("ROS nodes have been terminated by Garbage Collection.");
		}
		session.close();
		getLogger().info("FRI Session was closed.");
		getLogger().info("ROS loop has ended. Application terminated.");
		super.dispose();
	}
}
