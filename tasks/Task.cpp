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
    if (_frame_in_left.read(input_frame)==RTT::NewData)
    {
 	if (_store_image_filename.read(filename)==RTT::NewData)
	{
            //left.init(input_frame->size.width, input_frame->size.height, input_frame->getDataDepth(), input_frame->getFrameMode());

            ::base::samples::frame::Frame *frame_left_ptr = frame_left.write_access();

            frame_left_ptr->init(*input_frame, true);
            
            //frame_left_ptr->init(input_frame->size.width, input_frame->size.height, input_frame->getDataDepth(), _output_format.value());
        
            //frame_left_ptr->init(input_frame->size.width, input_frame->size.height, input_frame->getDataDepth()/2.0, input_frame->getFrameMode());
        
            /** Undistort the images **/
            //frameHelperLeft.convert(*input_frame, *frame_left_ptr, 0, 0, frame_helper::INTER_LINEAR, true);
            frame_left_ptr->received_time = base::Time::now();

            /** Write the image into the output port **/
            frame_left.reset(frame_left_ptr);
            _left_frame.write(frame_left);
	   
            //frameHelperLeft.convert(left, *frame_left_ptr, 0, 0, frame_helper::INTER_LINEAR, true);
 
            char file[240];
            int acq_mode;
            sscanf (filename.c_str(), "%s %d", file, &acq_mode);
            if (acq_mode == 1){
                frameHelperLeft.saveFrame(file, *frame_left_ptr);
                std::cout << "storing image in: " << file << std::endl;
            }
	}
    }
    
    /** Read a new image in the port **/
    if (_frame_in_right.read(input_frame)==RTT::NewData)
    {
 	if (_store_image_filename.read(filename)==RTT::NewData)
	{
            //right.init(input_frame->size.width, input_frame->size.height, input_frame->getDataDepth(), input_frame->getFrameMode());

            ::base::samples::frame::Frame *frame_right_ptr = frame_right.write_access();

            frame_right_ptr->init(*input_frame, true);
            
            //frame_right_ptr->init(input_frame->size.width, input_frame->size.height, input_frame->getDataDepth(), _output_format.value());
        
            //frame_right_ptr->init(input_frame->size.width, input_frame->size.height, input_frame->getDataDepth()/2.0, input_frame->getFrameMode());

            /** Undistorted the images **/
            //frameHelperRight.convert(*input_frame, *frame_right_ptr, 0, 0, frame_helper::INTER_LINEAR, true);
            frame_right_ptr->received_time = base::Time::now();

            /** Write the image into the output port **/
            frame_right.reset(frame_right_ptr);
            _right_frame.write(frame_right);
	   
            //frameHelperRight.convert(right, *frame_right_ptr, 0, 0, frame_helper::INTER_LINEAR, true);
 
            char file[240];
            int acq_mode;
            sscanf (filename.c_str(), "%s %d", file, &acq_mode);
            if (acq_mode == 2){
                frameHelperRight.saveFrame(file, *frame_right_ptr);
                std::cout << "storing image in: " << file << std::endl;
            }
 	}
    }

    /** Read a new image in the port **/
    if (_frame_in.read(input_frame)==RTT::NewData)
    {
 	if (_store_image_filename.read(filename)==RTT::NewData)
	{
            left.init(input_frame->size.width, input_frame->size.height, input_frame->getDataDepth()/2.0, input_frame->getFrameMode());
            right.init(input_frame->size.width, input_frame->size.height, input_frame->getDataDepth()/2.0, input_frame->getFrameMode());
            
            /** De-Interlace the images **/
            deInterlace(*input_frame, left, right);

            ::base::samples::frame::Frame *frame_left_ptr = frame_left.write_access();
            ::base::samples::frame::Frame *frame_right_ptr = frame_right.write_access();

            frame_left_ptr->init(left, true);
            frame_right_ptr->init(right, true);

            //frame_left_ptr->init(input_frame->size.width, input_frame->size.height, input_frame->getDataDepth()/2.0, _output_format.value());
            //frame_right_ptr->init(input_frame->size.width, input_frame->size.height, input_frame->getDataDepth()/2.0, _output_format.value());
        
            //frame_left_ptr->init(input_frame->size.width, input_frame->size.height, input_frame->getDataDepth()/2.0, input_frame->getFrameMode());
            //frame_right_ptr->init(input_frame->size.width, input_frame->size.height, input_frame->getDataDepth()/2.0, input_frame->getFrameMode());
            
            /** Undistorted the images **/
            //frameHelperLeft.convert(left, *frame_left_ptr, 0, 0, frame_helper::INTER_LINEAR, true);
            //frameHelperRight.convert(right, *frame_right_ptr, 0, 0, frame_helper::INTER_LINEAR, true);
            frame_left_ptr->received_time = input_frame->time;
            frame_right_ptr->received_time = frame_left_ptr->received_time;

            /** Write the image into the output port **/
            frame_left.reset(frame_left_ptr);
            _left_frame.write(frame_left);
            frame_right.reset(frame_right_ptr);
            _right_frame.write(frame_right);
	   
            //frameHelperLeft.convert(left, *frame_left_ptr, 0, 0, frame_helper::INTER_LINEAR, true);
            //frameHelperRight.convert(right, *frame_right_ptr, 0, 0, frame_helper::INTER_LINEAR, true);
 
            char file[240];
            int acq_mode;
            sscanf (filename.c_str(), "%s %d", file, &acq_mode);
            if (acq_mode == 1){
                frameHelperLeft.saveFrame(file, *frame_left_ptr);
                std::cout << "storing image in: " << file << std::endl;
            }
            else if (acq_mode == 2){
                frameHelperRight.saveFrame(file, *frame_right_ptr);
                std::cout << "storing image in: " << file << std::endl;
            }
            else if (acq_mode == 3){
                char stereoleftfile[240];
                sprintf (stereoleftfile, "%s_left.png", file);
                frameHelperLeft.saveFrame(stereoleftfile, *frame_left_ptr);
                std::cout << "storing image in: " << stereoleftfile << std::endl;
                char stereorightfile[240];
                sprintf (stereorightfile, "%s_right.png", file);
                frameHelperRight.saveFrame(stereorightfile, *frame_right_ptr);
                std::cout << "storing image in: " << stereorightfile << std::endl;
            }
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


