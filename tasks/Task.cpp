/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Task.hpp"

using namespace camera_bb2;

Task::Task(std::string const& name)
    : TaskBase(name)
{
}

Task::Task(std::string const& name, RTT::ExecutionEngine* engine)
    : TaskBase(name, engine)
{
}

Task::~Task()
{
}

/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See Task.hpp for more detailed
// documentation about them.

bool Task::configureHook()
{
    if (! TaskBase::configureHook())
        return false;

    /** Read the camera calibration parameters **/
    stereo_calibration = _calibration_parameters.value();

    /** Frame Helper **/
    frameHelperLeft.setCalibrationParameter(stereo_calibration.camLeft);
    frameHelperRight.setCalibrationParameter(stereo_calibration.camRight);

    /** Left and Right Frames **/
    ::base::samples::frame::Frame *leftFrame = new ::base::samples::frame::Frame();
    ::base::samples::frame::Frame *rightFrame = new ::base::samples::frame::Frame();
    frame_left.reset(leftFrame);
    frame_right.reset(rightFrame);
    leftFrame = NULL; rightFrame = NULL;

    index_frame=0;

    return true;
}

bool Task::startHook()
{
    if (! TaskBase::startHook())
        return false;
    return true;
}

void Task::updateHook()
{

    RTT::extras::ReadOnlyPointer<base::samples::frame::Frame> input_frame;  //! Input port frames
    ::base::samples::frame::Frame left, right;
    std::string filename;

    TaskBase::updateHook();

    /** Read a new image in the port **/
    if (_frame_in.read(input_frame)==RTT::NewData)
    {
        left.init(input_frame->size.width, input_frame->size.height, input_frame->getDataDepth()/2.0, input_frame->getFrameMode());
        right.init(input_frame->size.width, input_frame->size.height, input_frame->getDataDepth()/2.0, input_frame->getFrameMode());

        /** Interlace the images **/
        deInterlace(*input_frame, left, right);

        ::base::samples::frame::Frame *frame_left_ptr = frame_left.write_access();
        ::base::samples::frame::Frame *frame_right_ptr = frame_right.write_access();

        frame_left_ptr->init(input_frame->size.width, input_frame->size.height, input_frame->getDataDepth()/2.0, _output_format.value());
        frame_right_ptr->init(input_frame->size.width, input_frame->size.height, input_frame->getDataDepth()/2.0, _output_format.value());

        /** Undistorted the images **/
        frameHelperLeft.convert(left, *frame_left_ptr, 0, 0, frame_helper::INTER_LINEAR, true);
        frameHelperRight.convert(right, *frame_right_ptr, 0, 0, frame_helper::INTER_LINEAR, true);
        frame_left_ptr->received_time = base::Time::now();
        frame_right_ptr->received_time = frame_left_ptr->received_time;

        /** Write the image into the output port **/
        frame_left.reset(frame_left_ptr);
        _left_frame.write(frame_left);
        frame_right.reset(frame_right_ptr);
        _right_frame.write(frame_right);
	
	if (_acquire_image.read(filename)==RTT::NewData)
	{
	    frameHelperLeft.saveFrame(filename, *frame_left_ptr);
	}
    }
}

void Task::errorHook()
{
    TaskBase::errorHook();
}
void Task::stopHook()
{
    TaskBase::stopHook();
}
void Task::cleanupHook()
{
    TaskBase::cleanupHook();
}

void Task::deInterlace(const base::samples::frame::Frame &input, base::samples::frame::Frame &left, base::samples::frame::Frame &right)
{
    uint32_t width=input.size.width;
    uint32_t height=input.size.height;

    register int i = (width*height*2)-1;
    register int j = (width*height)-1;

    while (i > 0) {
        right.image[j] = input.image[i--];
        left.image[j--] = input.image[i--];
    }

    left.copyImageIndependantAttributes(input);
    right.copyImageIndependantAttributes(input);

}


