package cpi.tools.grip;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.stream.Collectors;
import java.util.HashMap;

import org.opencv.core.*;
import org.opencv.core.Core.*;
import org.opencv.features2d.FeatureDetector;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.*;
import org.opencv.objdetect.*;

/**
* GRIP3To1Switch class.
*
* <p>An OpenCV pipeline generated by GRIP.
*
* @author GRIP
*/
public class GRIP3X1SwitchPipeline {

	//Outputs
	private Mat switch0Output = new Mat();
	private Mat switch1Output = new Mat();
	private Mat resizeImageOutput = new Mat();

	private boolean switch0Switch = true;
	private boolean switch1Switch = true;
	static {
		System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
	}

	/**
	 * This is the primary method that runs the entire pipeline and updates the outputs.
	 */
	public void process(Mat source0, Mat source1,Mat source2) {
		// Step Switch0:
		//Inputs
		Mat switch0IfTrue = source0;
		Mat switch0IfFalse = source1;
		//Output
		Ref<Mat> switch0OutputRef = new Ref<Mat>();
		pipelineSwitch(switch0Switch, switch0IfTrue,
		switch0IfFalse, switch0OutputRef);
		//output assignment
		switch0Output = switch0OutputRef.get();
		// Step Switch1:
		//Inputs
		Mat switch1IfTrue = switch0Output;
		Mat switch1IfFalse = source2;
		//Output
		Ref<Mat> switch1OutputRef = new Ref<Mat>();
		pipelineSwitch(switch1Switch, switch1IfTrue,
		switch1IfFalse, switch1OutputRef);
		//output assignment
		switch1Output = switch1OutputRef.get();
		// Step Resize_Image0:
		Mat resizeImageInput = switch1Output;
		double resizeImageWidth = 640.0;
		double resizeImageHeight = 480.0;
		int resizeImageInterpolation = Imgproc.INTER_CUBIC;
		resizeImage(resizeImageInput, resizeImageWidth, resizeImageHeight, resizeImageInterpolation, resizeImageOutput);

	}

	/**
	 * This method is a generated setter for the condition of Switch
	 * @param the condition to set
	 */
	 public void setswitch0(boolean value) {
	 	switch0Switch = value;
	 }

	/**
	 * This method is a generated setter for the condition of Switch
	 * @param the condition to set
	 */
	 public void setswitch1(boolean value) {
	 	switch1Switch = value;
	 }

	/**
	 * This method is a generated getter for the output of a Switch.
	 * @return Mat output from Switch.
	 */
	public Mat switch0Output() {
		return switch0Output;
	}

	/**
	 * This method is a generated getter for the output of a Switch.
	 * @return Mat output from Switch.
	 */
	public Mat grip3X1Output() {
		return switch1Output;
	}

	/**
	 * This method is a generated getter for the output of a Resize_Image.
	 * @return Mat output from Resize_Image.
	 */
	public Mat resizeImageOutput() {
		return resizeImageOutput;
	}


	/**
	 * Selects an output from two inputs based on a boolean.
	 * @param sw The boolean that determines the output.
	 * @param onTrue The output if sw is true.
	 * @param onFalse The output if sw is false.
	 * @param output The output which is equal to either onTrue or onFalse.
	 */
	private <T> void pipelineSwitch(boolean sw, T onTrue, T onFalse, Ref<T> output) {
		if (sw) {
			output.set(onTrue);
		}
		else {
			output.set(onFalse);
		}
	}

	/**
	 * Scales and image to an exact size.
	 * @param input The image on which to perform the Resize.
	 * @param width The width of the output in pixels.
	 * @param height The height of the output in pixels.
	 * @param interpolation The type of interpolation.
	 * @param output The image in which to store the output.
	 */
	private void resizeImage(Mat input, double width, double height,
		int interpolation, Mat output) {
		Imgproc.resize(input, output, new Size(width, height), 0.0, 0.0, interpolation);
	}


	/**
	 * Enables C-style output parameters in Java to avoid creating custom data classes for each
	 * operation.
	 *
	 * <p>Syntax is {@code Ref<T> varName = new Ref<T>(initValue)}.
	 * Where varName is the name of the variable and initValue is of type T and contains initial value.
	 * </p>
	 * @param <T> The type of object being referenced
	 */
	private static class Ref<T> {
		private T value;

		/**
		 * Constructor for a Ref object.
		 * @param initValue Type T initial value for the object.
		 */
		public Ref(T initValue) {
			value = initValue;
		}

		/**
		 * Constructor for a Ref object without an initial value.
		 * Equivalent to calling Ref(null)
		 */
		public Ref() {
			this(null);
		}

		/**
		 * Sets the object to contain a new value.
		 *
		 * @param newValue the new value being referenced
		 */
		public void set(T newValue) {
			value = newValue;
		}

		/**
		 * Gets the current referenced value
		 *
		 * @return the current referenced value
		 */
		public T get() {
			return value;
		}
	}

}

