#include "acuavvs.h"

#include "uavvs_c_types.h"
#include "codels.h"

/* --- Task main -------------------------------------------------------- */

static vpColVector  ve_zero = vpColVector(6,0); // To be used to compare with the velocity 

/** Codel vs_start of task main.
 *
 * Triggered by uavvs_start.
 * Yields to uavvs_perm.
 */
genom_event
vs_start(uavvs_ids *ids, const uavvs_desired *desired,
         const genom_context self)
{
  // Initializing port out 'desired' by setting all the flags to false
  or_rigid_body_state *ddata;

  ddata = desired->data(self);
  ddata->ts.sec = 0;
  ddata->ts.nsec = 0;
  ddata->intrinsic = false;
  ddata->pos._present = false;
  ddata->att._present = false;
  ddata->vel._present = false;
  ddata->avel._present = false;
  ddata->acc._present = false;
  ddata->aacc._present = false;
  ddata->jerk._present = false;
  ddata->snap._present = false;
  desired->write(self);

  // Default flags set to false
  ids->display = false;
  ids->grabber = false;
  ids->detector = false;
  ids->camera = false;
  ids->servo = false;

  ids->I = new uavvs_image_s;
  
  /* init logging */
  ids->log = new uavvs_log_s;
  if (!ids->log) abort();

  ids->log->req.aio_fildes = -1;
  ids->log->req.aio_offset = 0;
  ids->log->req.aio_buf = ids->log->buffer;
  ids->log->req.aio_nbytes = 0;
  ids->log->req.aio_reqprio = 0;
  ids->log->req.aio_sigevent.sigev_notify = SIGEV_NONE;
  ids->log->req.aio_lio_opcode = LIO_NOP;
  ids->log->pending = false;
  ids->log->skipped = false;
  ids->log->decimation = 1;
  ids->log->missed = 0;
  ids->log->total = 0;

  return uavvs_perm;
}


/** Codel vs_loop of task main.
 *
 * Triggered by uavvs_perm.
 * Yields to uavvs_pause_perm.
 */
genom_event
vs_loop(uavvs_ids *ids, const uavvs_desired *desired,
        uavvs_log_s **log, const genom_context self)
{
double t = vpTime::measureTimeMs();

  // to be able to read what was published on the port `desired`, we read the content in `or_rigid_body_state`
  // data type (The type of the port)
  or_rigid_body_state *ddata;
  ddata = desired->data(self);

  // Get the last timestamp to be able to calculate the task's loop time from gettimeofday() (used for acceleration calculation)
  double last_ts = ddata->ts.sec + 1e-9 * ddata->ts.nsec;

  // Get the current time (when grabbing the image) to be used as timestamp for velocity command
  struct timeval tv;
  gettimeofday(&tv,NULL);
  
  // Calculating the task's loop time (should be approximately 50ms)
  double delta_t;
  if(ids->servo)
    delta_t = (tv.tv_sec + 1e-6 * tv.tv_usec) - last_ts;
  else // To make sure that at the beginning this delta_t is not negative, we fix it at main task's period
    delta_t = uavvs_loop_period_ms;
  
  // If no grabber initialized, don't even try to do anything in the loop.
  if(ids->grabber)
  {
    // Grabbed an image ==> increment frame_count
    ids->g->frame_count++;

    // Grabbing image
    switch(ids->g->device)
    {
      case 0:
        ids->g->grabber.acquire(ids->I->data); break;

      case 1:
        ids->g->cap >> ids->g->frame;
        vpImageConvert::convert(ids->g->frame, ids->I->data);break;

      default: break;
    }

    // If display not enabled, publish this message for the first 10 frames only (otherwise the screen will be full with ok)
    /*if(!ids->display && ids->grabber && ids->g->frame_count < 10)
      std::cout << "Ok." << std::endl;
    */

    // Displaying if needed
    if(ids->display && (ids->g->frame_count % ids->d->display_count == 0))
      vpDisplay::display(ids->I->data);

    // Initialize the display window corresponding for the image I for one time
    if(ids->display && !ids->d->display->isInitialised())
    {
      std::cout << "Display initialized" << std::endl;
      ids->d->display->init(ids->I->data);
    }


    // If detector and camera initialized, start detecting apriltag
    if(ids->detector && ids->camera)
    {
      ids->apriltag_detector->cMo_vec.clear();
      ids->apriltag_detector->d.detect(ids->I->data, ids->tag_size, ids->cam->data, ids->apriltag_detector->cMo_vec);

      for (size_t i = 0; i < ids->apriltag_detector->d.getNbObjects(); i++) // if there is a tag in the image
      { 
        std::vector<vpImagePoint> p = ids->apriltag_detector->d.getPolygon(i);
        if(ids->apriltag_detector->d.getNbObjects() > 1) // Draw the bounding box only if more than 1 tag detected
        {
          vpRect bbox = ids->apriltag_detector->d.getBBox(i);
          if(ids->display && (ids->g->frame_count % ids->d->display_count == 0))
            vpDisplay::displayRectangle(ids->I->data, bbox, vpColor::green);
        }
        std::string message = ids->apriltag_detector->d.getMessage(i);
        std::size_t tag_id_pos = message.find("id: ");

        if(ids->display && (ids->g->frame_count % ids->d->display_count == 0))
        {  
          for (size_t j = 0; j < p.size(); j++) {
            vpDisplay::displayCross(ids->I->data, p[j], 14, vpColor::red, 3);
          }
        }
      }

      // VISUAL SERVOING
      if(ids->servo)
      {
        if (ids->apriltag_detector->d.getNbObjects() == 1) // if 1 tag is detected, start servoing
        {
          // Update current points used to compute the moments
          std::vector<vpImagePoint> vec_ip = ids->apriltag_detector->d.getPolygon(0);
          ids->task->vec_P.clear();
          for (size_t i = 0; i < vec_ip.size(); i++) { // size = 4
            double x = 0, y = 0;
            vpPixelMeterConversion::convertPoint(ids->cam->data, vec_ip[i], x, y);
            vpPoint P;
            P.set_x(x);
            P.set_y(y);
            ids->task->vec_P.push_back(P);
          }

          // Current moments
          ids->task->m_obj.setType(vpMomentObject::DENSE_POLYGON); // Consider the AprilTag as a polygon
          ids->task->m_obj.fromVector(ids->task->vec_P);                      // Initialize the object with the points coordinates

          ids->task->mg.linkTo(ids->task->mdb);       // Add gravity center to database
          ids->task->mc.linkTo(ids->task->mdb);       // Add centered moments to database
          ids->task->man.linkTo(ids->task->mdb);      // Add area normalized to database
          ids->task->mgn.linkTo(ids->task->mdb);      // Add gravity center normalized to database
          ids->task->mdb.updateAll(ids->task->m_obj); // All of the moments must be updated, not just an_d
          ids->task->mg.compute();         // Compute gravity center moment
          ids->task->mc.compute();         // Compute centered moments AFTER gravity center
          
          ids->task->man.setDesiredArea(ids->task->area); // Desired area was init at 0 (unknow at contruction), need to be updated here
          ids->task->man.compute();            // Compute area normalized moment AFTER centered moment
          ids->task->mgn.compute();            // Compute gravity center normalized moment AFTER area normalized moment

          ids->task->s_mgn.update(ids->task->A, ids->task->B, ids->task->C);
          ids->task->s_mgn.compute_interaction();
          ids->task->s_man.update(ids->task->A, ids->task->B, ids->task->C);
          ids->task->s_man.compute_interaction();


          /* Sort points from their height in the image, and keep original indexes.
          This is done once, in order to be independent from the orientation of the tag
          when detecting vanishing points. */
          if (!ids->task->vec_ip_has_been_sorted) {
            for (size_t i = 0; i < vec_ip.size(); i++) {

              // Add the points and their corresponding index
              std::pair<size_t, vpImagePoint> index_pair = std::pair<size_t, vpImagePoint>(i, vec_ip[i]);
              ids->task->vec_ip_sorted.push_back(index_pair);
            }

            // Sort the points and indexes from the v value of the points
            std::sort(ids->task->vec_ip_sorted.begin(), ids->task->vec_ip_sorted.end(), compareImagePoint);

            ids->task->vec_ip_has_been_sorted = true;
          }

          // Use the two highest points for the first line, and the two others for the second line.
          vpFeatureBuilder::create(ids->task->s_vp, ids->cam->data, vec_ip[ids->task->vec_ip_sorted[0].first], vec_ip[ids->task->vec_ip_sorted[1].first],
                                    vec_ip[ids->task->vec_ip_sorted[2].first], vec_ip[ids->task->vec_ip_sorted[3].first],
                                    vpFeatureVanishingPoint::selectAtanOneOverRho());
                        
          ids->task->t.set_cVe(ids->task->cVe);
          ids->task->t.set_eJe(ids->task->eJe);

          // Compute the control law. Velocities are computed in the mobile robot reference frame
          ids->task->ve = ids->task->t.computeControlLaw();

          // A tag is detected in this section (if...) so we should set the tag_detected flag
          // to true          
          ids->I->tag_detected = true;

          if(ids->display && (ids->g->frame_count % ids->d->display_count == 0))
          {
            for (size_t i = 0; i < 4; i++) {
              vpDisplay::displayCross(ids->I->data, vec_ip[i], 15, vpColor::red, 1);
              std::stringstream ss;
              ss << i;
              vpDisplay::displayText(ids->I->data, vec_ip[i] + vpImagePoint(15, 15), ss.str(), vpColor::green);
            }

            // Display visual features
            vpDisplay::displayPolygon(ids->I->data, vec_ip, vpColor::green, 3); // Current polygon used to compure an moment
            vpDisplay::displayCross(ids->I->data, ids->apriltag_detector->d.getCog(0), 15, vpColor::green,
                                    3); // Current polygon used to compute a moment
            vpDisplay::displayLine(ids->I->data, 0, static_cast<int>(ids->I->data.getWidth() / 2.), static_cast<int>(ids->I->data.getHeight()) - 1,
                                    static_cast<int>(ids->I->data.getWidth() / 2.), vpColor::red,
                                    3); // Vertical line as desired x position
            vpDisplay::displayLine(ids->I->data, static_cast<int>(ids->I->data.getHeight() / 2.0), 0, static_cast<int>(ids->I->data.getHeight() / 2.),
                                    static_cast<int>(ids->I->data.getWidth()) - 1, vpColor::red,
                                    3); // Horizontal line as desired y position
  
            // Display lines corresponding to the vanishing point for the horizontal lines
            vpDisplay::displayLine(ids->I->data, vec_ip[ids->task->vec_ip_sorted[0].first], vec_ip[ids->task->vec_ip_sorted[1].first], vpColor::red, 1,
                                    false);
            vpDisplay::displayLine(ids->I->data, vec_ip[ids->task->vec_ip_sorted[2].first], vec_ip[ids->task->vec_ip_sorted[3].first], vpColor::red, 1,
                                    false);
          }
        }

        else // No tag is detected for servoing
          ids->I->tag_detected = false;

        if(!ids->I->tag_detected) // if no tag is detected, set current, old velocities and acceleration to 0
          ids->task->ve = 0;

      } // End if (servo)
    } // End if (detector & camera)
  } // End if (grabber)

  // Measure the time taken to finish the loop (in all cases, even if nothing is done)
  t = vpTime::measureTimeMs() - t;
  std::stringstream ss;
  //if(ids->I->tag_detected) // In case a tag is detected, print this string
  //  ss << "Detection time: " << t << " ms for " << ids->apriltag_detector->d.getNbObjects() << " tags";
  //else // Otherwise just print the loop time
  ss << "Loop time: " << t << "ms";

  /*if(!ids->display)
    std::cout << "t = " << t << " ms" << std::endl;
  */

  if(ids->grabber && ids->display && (ids->g->frame_count % ids->d->display_count == 0))
    vpDisplay::displayText(ids->I->data, 40, 20, ss.str(), vpColor::red);

  // If the display is enabled, flush 
  if(ids->grabber && ids->display && (ids->g->frame_count % ids->d->display_count == 0))
    vpDisplay::flush(ids->I->data);

  // Updating output ports
  // Port [desired] of type [rigid body]
  /*
    Update command vel port only when servo task is up and running
    Otherwise (before initialization) no need to update because it
    is fixed at 0
  */

  // timestamp is calculated when grabbing the image
  ddata->ts.sec = tv.tv_sec;
  ddata->ts.nsec = tv.tv_usec * 1000;

  if(ids->servo)
  { 
    // SEE rigid_body.gen file in /opt/share/idl/openrobots2-idl/pose/rigid_body.gen //  
    // velocity and acceleration in local frame (intrinsic = true) 
    ddata->intrinsic = true; 
    ddata->vel._present = true;
    ddata->vel._value.vx = ids->task->ve[0];
    ddata->vel._value.vy = ids->task->ve[1];
    ddata->vel._value.vz = ids->task->ve[2];
    ddata->avel._present = true;
    ddata->avel._value.wx = ids->task->ve[3];
    ddata->avel._value.wy = ids->task->ve[4];
    ddata->avel._value.wz = ids->task->ve[5];

    desired->write(self);

    // Logging
    uavvs_main_log(*ddata, delta_t, ids->I->tag_detected, *log);
  }

  // If finished before the end of the task's period, just wait
  return uavvs_pause_perm;
}


/** Codel vs_stop of task main.
 *
 * Triggered by uavvs_stop.
 * Yields to uavvs_ether.
 */
genom_event
vs_stop(uavvs_ids *ids, const genom_context self)
{
  if(ids->g != NULL)
  {
    // Closing the image grabber 
    switch(ids->g->device){
      case 0:
        ids->g->grabber.disconnect(); break;

      case 1:
        ids->g->cap.release(); break;

      default:
        break;
    }
  }

  return uavvs_ether;
}
