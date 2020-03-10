#include "acuavvs.h"

#include "uavvs_c_types.h"
#include "codels.h"

/* --- Function enable_display ------------------------------------------ */

/** Codel uavvs_enable_display of function enable_display.
 *
 * Returns genom_ok.
 */
genom_event
uavvs_enable_display(uavvs_display_s **d, bool *display,
                     double frequency, const uavvs_grabber_s *g,
                     const genom_context self)
{
  // Display flag true because created 
  *display = true;

  (*d) = new uavvs_display_s;

  // Set the frequency of the display
  if(g == NULL) // grabber not yet initialized, set the display frequency to 1Hz
  {
    std::cout << "Grabber not yet initialized, setting the display frequency to default: 1Hz" << std::endl;
    (*d)->display_count = 50;
    std::cout << "Display succesfully initialized with 1 image per second " << std::endl;
  }

  else // grabber is initialized
  {
    (*d)->display_count = g->frame_rate / frequency;
    std::cout << "Display succesfully initialized with " << frequency << " image per second " << std::endl;
  }
    
  (*d)->display = new vpDisplayX();

  return genom_ok;
}


/* --- Function close_display ------------------------------------------- */

/** Codel uavvs_close_display of function close_display.
 *
 * Returns genom_ok.
 */
genom_event
uavvs_close_display(uavvs_display_s **d, bool *display,
                    uavvs_image_s **I, const genom_context self)
{

  *display = false;
  
  (*d)->display->close((*I)->data);
  delete (*d);
  std::cout << "Display successfully closed" << std::endl;

  return genom_ok;
}


/* --- Function init_grabber -------------------------------------------- */

/** Codel uavvs_init_grabber of function init_grabber.
 *
 * Returns genom_ok.
 */
genom_event
uavvs_init_grabber(uavvs_grabber_s **g, bool *grabber,
                   uavvs_image_s **I, int16_t device,
                   int16_t cam_index, int16_t frame_rate, bool shutter,
                   bool gain, const genom_context self)
{
  // Grabber flag true because created
  *grabber = true;

  (*g) = new uavvs_grabber_s;
  (*g)->frame_count = 0;
  (*g)->frame_rate = frame_rate;
  (*g)->device = device; // Variable to know if we are using FlyCap or OpenCV (For debug purposes)

  if(device == 0)
  {
    #ifdef VISP_HAVE_FLYCAPTURE
      (*g)->grabber.setCameraIndex(cam_index); // Default camera is the first on the bus
      (*g)->grabber.setFrameRate(frame_rate);
      (*g)->grabber.setShutter(shutter);
      (*g)->grabber.setGain(gain);
    #else
      std::cout << "Install Flycapture SDK, configure and build ViSP again." << std::endl;
    #endif
  }

  if(device == 1)
  {
    #ifdef VISP_HAVE_OPENCV
      (*g)->cap.open(cam_index);
    #else
      std::cout << "Can't find OpenCV libraries" << std::endl;
    #endif
  }

  std::cout << "Grabber successfully initialized" << std::endl;

  return genom_ok;
}


/* --- Function init_detector ------------------------------------------- */

/** Codel uavvs_init_detector of function init_detector.
 *
 * Returns genom_ok.
 */
genom_event
uavvs_init_detector(uavvs_detector_s **apriltag_detector,
                    double *tag_size, bool *detector, double tagSize,
                    double quad_decimate, double nThreads,
                    const genom_context self)
{
  // Detector flag true because created
  *detector = true;

  // Setting tag_size for use in the main loop
  *tag_size = tagSize;

  vpDetectorAprilTag::vpAprilTagFamily tagFamily = vpDetectorAprilTag::TAG_36h11;
  vpDetectorAprilTag::vpPoseEstimationMethod poseEstimationMethod = vpDetectorAprilTag::HOMOGRAPHY_VIRTUAL_VS;
  bool align_frame = false;

  (*apriltag_detector) = new uavvs_detector_s;
  (*apriltag_detector)->d.setAprilTagFamily(tagFamily);
  (*apriltag_detector)->d.setAprilTagQuadDecimate(quad_decimate);
  (*apriltag_detector)->d.setAprilTagPoseEstimationMethod(poseEstimationMethod);
  (*apriltag_detector)->d.setAprilTagNbThreads(nThreads);
  (*apriltag_detector)->d.setDisplayTag(false, vpColor::none, 2);
  (*apriltag_detector)->d.setZAlignedWithCameraAxis(align_frame);

  std::cout << "Detector succesfully initialized" << std::endl;

  return genom_ok;
}


/* --- Function init_camera --------------------------------------------- */

/** Codel uavvs_init_camera of function init_camera.
 *
 * Returns genom_ok.
 */
genom_event
uavvs_init_camera(uavvs_camera_s **cam, bool *camera, double px,
                  double py, double u0, double v0, double kud,
                  double kdu, const genom_context self)
{
  // Camera flag true because created
  *camera = true;

  (*cam) = new uavvs_camera_s;
  (*cam)->data.initPersProjWithDistortion(px, py, u0, v0, kud, kdu);

  std::cout << "Camera parameters succesfully initialized" << std::endl;

  return genom_ok;
}


/* --- Function init_servo ---------------------------------------------- */

/** Codel uavvs_init_servo of function init_servo.
 *
 * Returns genom_ok.
 */
genom_event
uavvs_init_servo(uavvs_servo_s **task, bool *servo, double tag_size,
                 double lambda_0, double lambda_inf,
                 double lambda_dot_0, double z_desired,
                 double camera_pan, const double camera_pos[3],
                 const genom_context self)
{
  /*
  At the launch of the main task, nothing will be initialized.
  So when this function is called (from Matlab basically) it will have to launch all the 
  objects used to do visual servoing.  
  Every other data is used only in this function and then is destroyed.
  */

  // Servo flag true because created
  *servo = true;

  (*task) = new uavvs_servo_s;
  (*task)->t.setServo(vpServo::EYEINHAND_L_cVe_eJe);
  (*task)->t.setInteractionMatrixType(vpServo::CURRENT);
  (*task)->t.setLambda(lambda_0, lambda_inf, lambda_dot_0);

  vpRxyzVector c1_rxyz_c2(0, camera_pan, 0);
  vpRotationMatrix c1Rc2(c1_rxyz_c2);                      // Rotation between camera 1 and 2
  vpHomogeneousMatrix c1Mc2(vpTranslationVector(), c1Rc2); // Homogeneous matrix between c1 and c2

  vpRotationMatrix c1Re{1, 0, 0, 0, 0, -1, 0, 1, 0}; // Rotation between camera 1 and E
  vpTranslationVector c1te(camera_pos[0], camera_pos[1], camera_pos[2]);            // Translation between camera 1 and E
  vpHomogeneousMatrix c1Me(c1te, c1Re);             // Homogeneous matrix between c1 and E

  vpHomogeneousMatrix c2Me = c1Mc2.inverse() * c1Me; // Homogeneous matrix between c2 and E

  (*task)->cVe.buildFrom(c2Me);
  (*task)->t.set_cVe((*task)->cVe);

  (*task)->eJe[0][0] = 1; (*task)->eJe[1][1] = 1;
  (*task)->eJe[2][2] = 1; (*task)->eJe[3][3] = 0;
  (*task)->eJe[4][4] = 0; (*task)->eJe[5][5] = 1;
      
  // Define the desired polygon corresponding the the AprilTag CLOCKWISE
  double X[4] = {tag_size / 2., tag_size / 2., -tag_size / 2., -tag_size / 2.};
  double Y[4] = {tag_size / 2., -tag_size / 2., -tag_size / 2., tag_size / 2.};
  std::vector<vpPoint> vec_P_d;

  for (int i = 0; i < 4; i++) {
      vpPoint P_d(X[i], Y[i], 0);
      vpHomogeneousMatrix cdMo(0, 0, z_desired, 0, 0, 0);
      P_d.track(cdMo); //
      vec_P_d.push_back(P_d);
  }

  (*task)->man.setDesiredDepth(z_desired);
  (*task)->man_d.setDesiredDepth(z_desired);

  // Desired moments
  (*task)->m_obj_d.setType(vpMomentObject::DENSE_POLYGON); // Consider the AprilTag as a polygon
  (*task)->m_obj_d.fromVector(vec_P_d);                    // Initialize the object with the points coordinates

  (*task)->mb_d.linkTo((*task)->mdb_d);       // Add basic moments to database
  (*task)->mg_d.linkTo((*task)->mdb_d);       // Add gravity center to database
  (*task)->mc_d.linkTo((*task)->mdb_d);       // Add centered moments to database
  (*task)->man_d.linkTo((*task)->mdb_d);      // Add area normalized to database
  (*task)->mgn_d.linkTo((*task)->mdb_d);      // Add gravity center normalized to database
  (*task)->mdb_d.updateAll((*task)->m_obj_d); // All of the moments must be updated, not just an_d
  (*task)->mg_d.compute();                    // Compute gravity center moment
  (*task)->mc_d.compute();                    // Compute centered moments AFTER gravity center

  if ((*task)->m_obj_d.getType() == vpMomentObject::DISCRETE)
      (*task)->area = (*task)->mb_d.get(2, 0) + (*task)->mb_d.get(0, 2);
  else
      (*task)->area = (*task)->mb_d.get(0, 0);

  // Update moment with the desired area
  (*task)->man_d.setDesiredArea((*task)->area);

  (*task)->man_d.compute(); // Compute area normalized moment AFTER centered moments
  (*task)->mgn_d.compute(); // Compute gravity center normalized moment AFTER area normalized moment

  // Desired plane
  (*task)->C = 1.0 / z_desired;

  // Add the features
  (*task)->t.addFeature((*task)->s_mgn, (*task)->s_mgn_d);
  (*task)->t.addFeature((*task)->s_man, (*task)->s_man_d);
  (*task)->t.addFeature((*task)->s_vp, (*task)->s_vp_d, vpFeatureVanishingPoint::selectAtanOneOverRho());

  // Update desired gravity center normalized feature
  (*task)->s_mgn_d.update((*task)->A, (*task)->B, (*task)->C);
  (*task)->s_mgn_d.compute_interaction();
  // Update desired area normalized feature
  (*task)->s_man_d.update((*task)->A, (*task)->B, (*task)->C);
  (*task)->s_man_d.compute_interaction();

  // Update desired vanishing point feature for the horizontal line
  (*task)->s_vp_d.setAtanOneOverRho(0);
  (*task)->s_vp_d.setAlpha(0);

  (*task)->vec_ip_has_been_sorted = false;

  std::cout << "Visual servoing task succesfully initialized" << std::endl;

  return genom_ok;
}


/* --- Function log ----------------------------------------------------- */

/** Codel uavvs_log of function log.
 *
 * Returns genom_ok.
 * Throws uavvs_e_sys.
 */
genom_event
uavvs_log(const char path[64], uint32_t decimation, uavvs_log_s **log,
          const genom_context self)
{
  int fd;

  fd = open(path, O_WRONLY|O_APPEND|O_CREAT|O_TRUNC, 0666);
  if (fd < 0) return uavvs_e_sys_error(path, self);

  if (write(fd, uavvs_log_header_fmt "\n", sizeof(uavvs_log_header_fmt)) < 0)
      return uavvs_e_sys_error(path, self);

  if ((*log)->req.aio_fildes >= 0) {
      close((*log)->req.aio_fildes);

      if ((*log)->pending)
      while (aio_error(&(*log)->req) == EINPROGRESS)
          /* empty body */;
  }

  (*log)->req.aio_fildes = fd;
  (*log)->pending = false;
  (*log)->skipped = false;
  (*log)->decimation = decimation < 1 ? 1 : decimation;
  (*log)->missed = 0;
  (*log)->total = 0;
  
  return genom_ok;
}


/* --- Function log_stop ------------------------------------------------ */

/** Codel uavvs_log_stop of function log_stop.
 *
 * Returns genom_ok.
 */
genom_event
uavvs_log_stop(uavvs_log_s **log, const genom_context self)
{
  if (*log && (*log)->req.aio_fildes >= 0)
  close((*log)->req.aio_fildes);
  (*log)->req.aio_fildes = -1;
  
  return genom_ok;
  return genom_ok;
}
