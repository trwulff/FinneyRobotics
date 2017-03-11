package org.usfirst.frc.team1405.robot.Vision;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.MjpegServer;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.DriverStation;

import org.opencv.core.Mat;
import org.opencv.core.MatOfKeyPoint;
import org.opencv.core.Scalar;
import org.opencv.features2d.Features2d;
import org.opencv.imgproc.Imgproc;
import edu.wpi.first.wpilibj.networktables.NetworkTable;

import edu.wpi.first.wpilibj.networktables.NetworkTable;

import org.usfirst.frc.team1405.robot.Vision.pipelines.BlobPipeline;
import org.usfirst.frc.team1405.robot.Vision.pipelines.GearPipeline;

public class Vision2017 {
	//Defaults
	static final boolean DEF_AUTO_CAMERA_X_AUTOEXPOSURE=true;	
	static final boolean DEF_AUTO_CAMERA_X_AUTO_WHITE_BALLANCE=true;
	static final double DEF_AUTO_CAMERA_X_BRIGHTNESS=0;	
	static final double DEF_AUTO_CAMERA_X_WHITE_BALLANCE=0;
	static final double DEF_AUTO_CAMERA_X_EXPOSURE=0;	
	
	static final boolean DEF_TELEOP_CAMERA_X_AUTOEXPOSURE=true;	
	static final boolean DEF_TELEOP_CAMERA_X_AUTO_WHITE_BALLANCE=true;
	static final double DEF_TELEOP_CAMERA_X_BRIGHTNESS=0;	
	static final double DEF_TELEOP_CAMERA_X_WHITE_BALLANCE=0;
	static final double DEF_TELEOP_CAMERA_X_EXPOSURE=0;	

	static final String DEF_SET_CAMERA_X_PARAMETERS="1";


	//End Defaults
	
	//Keys
	static final String MAIN_TABLE="Robot/Vision";

	static String CAMERA_ID_KEY3="Pipelines/Gear Placement/"+"Select Gear camera ID (0, 1, 2)";
	static String CAMERA_ID_KEY2="Pipelines/Gear Placement/"+"Select Gear camera ID (0, 1)";
	static String CAMERA_ID_KEY=CAMERA_ID_KEY3;
	
	static final String GEAR_PLACEMENT_TABLE_NAME="Robot/Vision/Pipelines/Gear Placement";
	static final String SHOOTING_BLOB="Robot/Vision/Pipelines/Shooting Blob";
	static final String SELECT_PIPELINE="Robot/Vision/Pipelines";
	static final String PIPELINE_ID_KEY=SELECT_PIPELINE+"/"+"Select (Gear Placement=0, Shooting Blob = 1)";

	static final String AUTO_CAMERA_X_AUTOEXPOSURE="/Selected Camera/Autonomous/Enable Autoexposure";	
	static final String AUTO_CAMERA_X_AUTO_WHITE_BALLANCE="/Selected Camera/Autonomous/Enable Auto White Ballance";
	static final String AUTO_CAMERA_X_BRIGHTNESS="/Selected Camera/Autonomous/Brightness";	
	static final String AUTO_CAMERA_X_WHITE_BALLANCE="/Selected Camera/Autonomous/White Ballance";
	static final String AUTO_CAMERA_X_EXPOSURE="/Selected Camera/Autonomous/White Ballance";	
	
	static final String TELEOP_CAMERA_X_AUTOEXPOSURE="/Selected Camera/Teleop/Enable Autoexposure";	
	static final String TELEOP_CAMERA_X_AUTO_WHITE_BALLANCE="/Selected Camera/Teleop/Enable Auto White Ballance";
	static final String TELEOP_CAMERA_X_BRIGHTNESS="/Selected Camera/Teleop/Brightness";	
	static final String TELEOP_CAMERA_X_WHITE_BALLANCE="/Selected Camera/Teleop/White Ballance";
	static final String TELEOP_CAMERA_X_EXPOSURE="/Selected Camera/Teleop/White Ballance";	
	
	static final String SET_CAMERA_X_PARAMETERS="/Selected Camera/Choose selected camera parameters (Auto-0, Teleop=1";

	// End Keys
	
	static Thread visionThread;
	static GearPipeline gearPipeline=new GearPipeline(GEAR_PLACEMENT_TABLE_NAME);
	static BlobPipeline blobPipeline = new BlobPipeline(SHOOTING_BLOB);
	static final int VERT_RES=120;
	static final int HOR_RES=160;

	static CameraServer cameraServer;
	static MjpegServer outputStreamServer;
	static UsbCamera camera[]=new UsbCamera[3];
	static UsbCamera camera0;
	static UsbCamera camera1;
	static UsbCamera camera2;

	static CvSink cvSink[]=new CvSink[3];
	static CvSink cvSink0;
	static CvSink cvSink1;
	static CvSink cvSink2;
	
	static CvSource outputStream;


	static boolean enableCamera1=true;
	static boolean enableCamera2=true;
	static boolean enableCameraSwitch=true;
	
	static String cameraID="0";
	static NetworkTable table;
	static String pipelineID="0";

	static Mat rgb = new Mat();
	static Mat outputImage = new Mat();
	static MatOfKeyPoint keypoints = new MatOfKeyPoint();
	
	public static void robotInit(int numberOfCameras){
		
		if(numberOfCameras==1){
			enableCamera1=false;
			enableCamera2=false;
			enableCameraSwitch=false;
		}
		if(numberOfCameras==2){
			enableCamera1=true;
			enableCamera2=false;
			enableCameraSwitch=true;
			CAMERA_ID_KEY=CAMERA_ID_KEY2;
			
		}
		robotInit();
	}
	static public double[] getX(){
		return gearPipeline.getCenterX();
	}
	static public double[] getY(){
		return gearPipeline.getCenterY();
	}
	static public double[] getWidth(){
		return gearPipeline.getWidth();
	}
	static public double[] getHeight(){
		return gearPipeline.getHeight();
	}
	
	
	static public void robotInit(){
		table=NetworkTable.getTable(MAIN_TABLE);
		
		table.putBoolean(AUTO_CAMERA_X_AUTOEXPOSURE, table.getBoolean(AUTO_CAMERA_X_AUTOEXPOSURE, DEF_AUTO_CAMERA_X_AUTOEXPOSURE));
		table.putBoolean(AUTO_CAMERA_X_AUTO_WHITE_BALLANCE, table.getBoolean(AUTO_CAMERA_X_AUTO_WHITE_BALLANCE, DEF_AUTO_CAMERA_X_AUTO_WHITE_BALLANCE));
		table.putNumber(AUTO_CAMERA_X_BRIGHTNESS, table.getNumber(AUTO_CAMERA_X_BRIGHTNESS, DEF_AUTO_CAMERA_X_BRIGHTNESS));
		table.putNumber(AUTO_CAMERA_X_EXPOSURE, table.getNumber(AUTO_CAMERA_X_EXPOSURE, DEF_AUTO_CAMERA_X_EXPOSURE));
		table.putNumber(AUTO_CAMERA_X_WHITE_BALLANCE, table.getNumber(AUTO_CAMERA_X_WHITE_BALLANCE, DEF_AUTO_CAMERA_X_WHITE_BALLANCE));		
		
		table.putBoolean(TELEOP_CAMERA_X_AUTOEXPOSURE, table.getBoolean(TELEOP_CAMERA_X_AUTOEXPOSURE, DEF_TELEOP_CAMERA_X_AUTOEXPOSURE));
		table.putBoolean(TELEOP_CAMERA_X_AUTO_WHITE_BALLANCE, table.getBoolean(TELEOP_CAMERA_X_AUTO_WHITE_BALLANCE, DEF_TELEOP_CAMERA_X_AUTO_WHITE_BALLANCE));
		table.putNumber(TELEOP_CAMERA_X_BRIGHTNESS, table.getNumber(TELEOP_CAMERA_X_BRIGHTNESS, DEF_TELEOP_CAMERA_X_BRIGHTNESS));
		table.putNumber(TELEOP_CAMERA_X_EXPOSURE, table.getNumber(TELEOP_CAMERA_X_EXPOSURE, DEF_TELEOP_CAMERA_X_EXPOSURE));
		table.putNumber(TELEOP_CAMERA_X_WHITE_BALLANCE, table.getNumber(TELEOP_CAMERA_X_WHITE_BALLANCE, DEF_TELEOP_CAMERA_X_WHITE_BALLANCE));

		table.setPersistent(AUTO_CAMERA_X_AUTOEXPOSURE);
		table.setPersistent(AUTO_CAMERA_X_AUTO_WHITE_BALLANCE);
		table.setPersistent(AUTO_CAMERA_X_BRIGHTNESS);
		table.setPersistent(AUTO_CAMERA_X_EXPOSURE);
		table.setPersistent(AUTO_CAMERA_X_WHITE_BALLANCE);
		table.setPersistent(TELEOP_CAMERA_X_AUTOEXPOSURE);
		table.setPersistent(TELEOP_CAMERA_X_AUTO_WHITE_BALLANCE);
		table.setPersistent(TELEOP_CAMERA_X_BRIGHTNESS);
		table.setPersistent(TELEOP_CAMERA_X_EXPOSURE);
		table.setPersistent(TELEOP_CAMERA_X_WHITE_BALLANCE);
		
		table.putString(SET_CAMERA_X_PARAMETERS, DEF_SET_CAMERA_X_PARAMETERS);
		if(enableCameraSwitch){
		cameraID=table.getString(CAMERA_ID_KEY,"0");
		
		table.setPersistent(CAMERA_ID_KEY);
		if(cameraID=="2"&&!enableCamera2)cameraID="0";
		if(!enableCamera1&&!enableCamera2)cameraID="0";
		table.putString(CAMERA_ID_KEY, cameraID);
		}
		pipelineID=table.getString(PIPELINE_ID_KEY,pipelineID);
		table.putString(PIPELINE_ID_KEY, pipelineID);
		visionThread = new Thread(() -> {
			camera[0]=CameraServer.getInstance().startAutomaticCapture(0);
			camera[0].setResolution(HOR_RES, VERT_RES);
			camera[0].setFPS(15);
			cvSink[0] = CameraServer.getInstance().getVideo(camera[0]);
			
			if(enableCamera1){
			camera[1]=CameraServer.getInstance().startAutomaticCapture(1);
			camera[1].setResolution(HOR_RES, VERT_RES);	
			camera[1].setFPS(15);		
			cvSink[1] = CameraServer.getInstance().getVideo(camera[1]);
			}

			if(enableCamera2){
			camera[2]=CameraServer.getInstance().startAutomaticCapture(2);
			camera[2].setResolution(HOR_RES, VERT_RES);
			camera[2].setFPS(15);
			cvSink[2] = CameraServer.getInstance().getVideo(camera[2]);
			}
			outputStream = CameraServer.getInstance().putVideo("selected view", VERT_RES, HOR_RES);
			// Mats are very memory expensive. Lets reuse this Mat.
			Mat mat = new Mat();
			// This cannot be 'true'. The program will never exit if it is. This
			// lets the robot stop this thread when restarting robot code or
			// deploying.
			while (!Thread.interrupted()) {
				if(DriverStation.getInstance().isDisabled()){
					switch(table.getString(SET_CAMERA_X_PARAMETERS, DEF_SET_CAMERA_X_PARAMETERS)){
					case "0":
						autonomousInit();
						break;
					default:
					case "1":
						telropInit();
						break;
					}
				}
				
				// Tell the CvSink to grab a frame from the camera and put it
				// in the source mat.  If there is an error notify the output.
				if(enableCameraSwitch)if(DriverStation.getInstance().isDisabled()){
					cameraID=table.getString(CAMERA_ID_KEY,"0");
					table.setPersistent(CAMERA_ID_KEY);
				}
				switch(cameraID){
				case "0":
				default:
				if (cvSink[0].grabFrame(mat) == 0 ) {
					// Send the output the error.
				outputStream.notifyError(cvSink[0].getError());
					// skip the rest of the current iteration
					continue;
					}
				break;
				
				case"1":
					if((enableCamera1)){
					if (cvSink[1].grabFrame(mat) == 0 ) {
					// Send the output the error.
						outputStream.notifyError(cvSink[1].getError());
					// skip the rest of the current iteration
					continue;
					}
					}else{
						cameraID="0";
						table.putString(CAMERA_ID_KEY, cameraID);
					}
				break;

				case"2":
					if((enableCamera2)){
					if (cvSink[2].grabFrame(mat) == 0 ) {
					// Send the output the error.
				outputStream.notifyError(cvSink[2].getError());
					// skip the rest of the current iteration
					continue;
					}
					}else{
						cameraID="0";
						table.putString(CAMERA_ID_KEY, cameraID);
					}
				}
				
				// Put a rectangle on the image
				if(!mat.empty()){
					
//				blobPipeline.process(mat);
				if(DriverStation.getInstance().isDisabled()){
					Mat selectedOutput;
					switch(pipelineID){
					default:
					case "0":
				gearPipeline.process(mat);
						selectedOutput=gearPipeline.selectedOutput();
						break;
					case "1":
						blobPipeline.process(mat);
						selectedOutput=blobPipeline.selectedOutput();
						break;
					}
					outputStream.putFrame(selectedOutput);
				}
				}
			}
		});
		visionThread.setDaemon(true);
		visionThread.start();
		
	}
	
	public void selectCamera(boolean cameraID){
	if(enableCameraSwitch)if(DriverStation.getInstance().isDisabled()){
		if(cameraID){
			table.putString(CAMERA_ID_KEY,"0");
		}else{
			table.putString(CAMERA_ID_KEY,"1");
		}
	}
	}
	
	public static void autonomousInit(){
		int id=Integer.parseInt(cameraID);
		if(table.getBoolean(AUTO_CAMERA_X_AUTOEXPOSURE, DEF_AUTO_CAMERA_X_AUTOEXPOSURE))camera[id].setExposureAuto();
		if(table.getBoolean(AUTO_CAMERA_X_AUTO_WHITE_BALLANCE, DEF_AUTO_CAMERA_X_AUTO_WHITE_BALLANCE))camera[id].setWhiteBalanceAuto();
		camera[id].setBrightness((int)table.getNumber(AUTO_CAMERA_X_BRIGHTNESS, DEF_AUTO_CAMERA_X_BRIGHTNESS));
		camera[id].setExposureManual((int)table.getNumber(AUTO_CAMERA_X_EXPOSURE, DEF_AUTO_CAMERA_X_EXPOSURE));
		camera[id].setWhiteBalanceManual((int)table.getNumber(AUTO_CAMERA_X_WHITE_BALLANCE, DEF_AUTO_CAMERA_X_WHITE_BALLANCE));
	}	
	public static void telropInit(){
		int id=Integer.parseInt(cameraID);
		if(table.getBoolean(TELEOP_CAMERA_X_AUTOEXPOSURE, DEF_TELEOP_CAMERA_X_AUTOEXPOSURE))camera[id].setExposureAuto();
		if(table.getBoolean(TELEOP_CAMERA_X_AUTO_WHITE_BALLANCE, DEF_TELEOP_CAMERA_X_AUTO_WHITE_BALLANCE))camera[id].setWhiteBalanceAuto();
		camera[id].setBrightness((int)table.getNumber(TELEOP_CAMERA_X_BRIGHTNESS, DEF_TELEOP_CAMERA_X_BRIGHTNESS));
		camera[id].setExposureManual((int)table.getNumber(TELEOP_CAMERA_X_EXPOSURE, DEF_TELEOP_CAMERA_X_EXPOSURE));
		camera[id].setWhiteBalanceManual((int)table.getNumber(TELEOP_CAMERA_X_WHITE_BALLANCE, DEF_TELEOP_CAMERA_X_WHITE_BALLANCE));
		
	}

}
