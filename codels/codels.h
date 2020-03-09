#ifndef CODELS_H
#define CODELS_H

#include "uavvs_c_types.h"

#include <sys/time.h>
#include <aio.h>
#include <err.h>
#include <unistd.h>
#include <fcntl.h>


#include <visp3/core/vpImage.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/core/vpImageConvert.h>
#include <visp3/sensor/vpFlyCaptureGrabber.h>
#include <visp3/detection/vpDetectorAprilTag.h>
#include <visp3/core/vpXmlParserCamera.h>
#include <visp3/core/vpPixelMeterConversion.h>
#include <visp3/vs/vpServo.h>
#include <visp3/core/vpMomentObject.h>
#include <visp3/core/vpMomentDatabase.h>
#include <visp3/core/vpMomentBasic.h>
#include <visp3/core/vpMomentGravityCenter.h>
#include <visp3/core/vpMomentCentered.h>
#include <visp3/core/vpMomentAreaNormalized.h>
#include <visp3/core/vpMomentGravityCenterNormalized.h>
#include <visp3/visual_features/vpFeatureMomentGravityCenterNormalized.h>
#include <visp3/visual_features/vpFeatureMomentAreaNormalized.h>
#include <visp3/visual_features/vpFeatureVanishingPoint.h>
#include <visp3/visual_features/vpFeatureBuilder.h>

struct uavvs_display_s
{
    long display_count;
    vpDisplay *display;
};

struct uavvs_grabber_s
{
    long frame_count;
    long frame_rate;
    uint16_t device;

    #ifdef VISP_HAVE_FLYCAPTURE
      // FlyCapture grabber object
      vpFlyCaptureGrabber grabber;
    #endif

    #ifdef VISP_HAVE_OPENCV   
      // OpenCV grabber
      cv::VideoCapture cap;
      cv::Mat frame;
    #endif 
};

struct uavvs_image_s
{
    vpImage<unsigned char> data;
    bool tag_detected;
};

struct uavvs_detector_s
{
    vpDetectorAprilTag d;
    std::vector<vpHomogeneousMatrix> cMo_vec;
};

struct uavvs_camera_s
{
    vpCameraParameters data;
};

struct uavvs_servo_s
{
    // Servo object
    vpServo t;

    double area = 0;

    // Desired plane
    double A = 0.0;
    double B = 0.0;
    double C = 1;

    bool vec_ip_has_been_sorted;
    std::vector<std::pair<size_t, vpImagePoint> > vec_ip_sorted;
    std::vector<vpPoint> vec_P;

    vpVelocityTwistMatrix cVe;
    vpMatrix eJe = vpMatrix(6, 6, 0);

    vpColVector ve = vpColVector(6); // for linear and angular velocities
    //vpColVector ve_old = vpColVector(6); // for last linear and angular velocities

    //vpColVector ae = vpColVector(6); // for linear and angular accelerations

    // Momemnts
    vpMomentObject m_obj = vpMomentObject(3);
    vpMomentObject m_obj_d = vpMomentObject(3);
    vpMomentDatabase mdb, mdb_d;
    vpMomentBasic mb_d; // Here only to get the desired area m00
    vpMomentGravityCenter mg, mg_d;
    vpMomentCentered mc, mc_d;
    vpMomentAreaNormalized man = vpMomentAreaNormalized(0,0);
    vpMomentAreaNormalized man_d = vpMomentAreaNormalized(0,0);
    vpMomentGravityCenterNormalized mgn, mgn_d;

    // Features
    vpFeatureVanishingPoint s_vp, s_vp_d;
    vpFeatureMomentGravityCenterNormalized s_mgn = vpFeatureMomentGravityCenterNormalized(mdb, A, B, C);
    vpFeatureMomentGravityCenterNormalized s_mgn_d = vpFeatureMomentGravityCenterNormalized(mdb_d, A, B, C);
    vpFeatureMomentAreaNormalized s_man = vpFeatureMomentAreaNormalized(mdb, A, B, C);
    vpFeatureMomentAreaNormalized s_man_d = vpFeatureMomentAreaNormalized(mdb_d, A, B, C);
};

/*
 Log 
*/
static inline genom_event
uavvs_e_sys_error(const char *s, genom_context self)
{
  uavvs_e_sys_detail d;
  char buf[64], *p;

  d.code = errno;
#ifdef STRERROR_R_CHAR_P
  /* glibc managed to mess up with this function */
  p = strerror_r(d.code, buf, sizeof(buf));
#else
  char* c = strerror_r(d.code, buf, sizeof(buf));
  p = buf;
#endif
  snprintf(d.what, sizeof(d.what), "%s%s%s", s ? s : "", s ? ": " : "", p);

  return uavvs_e_sys(&d, self);
}

// Data structure
struct uavvs_log_s {
  struct aiocb req;
  char buffer[4096];
  bool pending, skipped;
  uint32_t decimation;
  size_t missed, total;

# define uavvs_logfmt	" %g"
# define uavvs_log_header_fmt                                                   \
  "ts dt "                                                                      \
  "nb_tags "                                                                    \
  "vx vy vz wx wy wz ax ay az aaz aay aaz"                                                  

# define uavvs_log_fmt                                                          \
  "%d.%09d"  uavvs_logfmt                                                       \
  " %d"                                                                         \
  uavvs_logfmt uavvs_logfmt uavvs_logfmt uavvs_logfmt uavvs_logfmt uavvs_logfmt \
  uavvs_logfmt uavvs_logfmt uavvs_logfmt uavvs_logfmt uavvs_logfmt uavvs_logfmt
};


// compareImagePoint is used in visual servoing part when sorting the polygon's points
inline bool compareImagePoint(std::pair<size_t, vpImagePoint> p1, std::pair<size_t, vpImagePoint> p2)
{
    return (p1.second.get_v() < p2.second.get_v());
};

/*
 * --- log -----------------------------------------------------------------
 */
static void
uavvs_main_log(const or_rigid_body_state &s, const double &dt, const bool &nbTags, uavvs_log_s *log)
{
  if (log->req.aio_fildes >= 0) {
    log->total++;
    if (log->total % log->decimation == 0) {
      if (log->pending) {
        if (aio_error(&log->req) != EINPROGRESS) {
          log->pending = false;
          if (aio_return(&log->req) <= 0) {
            warn("log");
            close(log->req.aio_fildes);
            log->req.aio_fildes = -1;
          }
        } else {
          log->skipped = true;
          log->missed++;
        }
      }
    }
  }

  if (log->req.aio_fildes >= 0 && !log->pending) {
    const double
      vx = s.vel._value.vx,
      vy = s.vel._value.vy,
      vz = s.vel._value.vz,
      wx = s.avel._value.wx,
      wy = s.avel._value.wy,
      wz = s.avel._value.wz,
      ax = s.acc._value.ax,
      ay = s.acc._value.ay,
      az = s.acc._value.az,
      aax = s.aacc._value.awx,
      aay = s.aacc._value.awy,
      aaz = s.aacc._value.awz;

    log->req.aio_nbytes = snprintf(
      log->buffer, sizeof(log->buffer),
      "%s" uavvs_log_fmt "\n",
      log->skipped ? "\n" : "",
      s.ts.sec, s.ts.nsec, dt,
      nbTags,
      vx, vy, vz, wx, wy, wz,
      ax, ay, az, aax, aay, aaz);

    if (aio_write(&log->req)) {
      warn("log");
      close(log->req.aio_fildes);
      log->req.aio_fildes = -1;
    } else
      log->pending = true;

    log->skipped = false;
  }
}


#endif