/* Particle Filter based Localization
 *
 */

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/message_filter.h>
#include <ros/subscriber.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <ros/package.h>


#include <opencv2/opencv.hpp>

#include <boost/random.hpp>

#include <cmath>

#include "pf_localization.h"

const double map_width  = 6.44;
const double map_height = 3.33;
const double nbeams = 5;

/** callback function for scan and base pose ground truth */
void PFLocalization::robot_callback( const sensor_msgs::LaserScanConstPtr & scan_msg,
                     const nav_msgs::OdometryConstPtr & base_pose_msg ) {
    // discretize the scan from 361 -> 5
    int step = (scan_msg->ranges.size()-1)/(nbeams-1);
    for ( int i = 0; i < nbeams; ++ i ) {
        scan_data_[i] = scan_msg->ranges[0+step*i];
    }

    // obtain the current pose to quaternion
    tf::Quaternion q;
    q.setX( base_pose_msg->pose.pose.orientation.x );
    q.setY( base_pose_msg->pose.pose.orientation.y );
    q.setZ( base_pose_msg->pose.pose.orientation.z );
    q.setW( base_pose_msg->pose.pose.orientation.w );

    // convert quaternion to rotation matrix
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY( roll, pitch, yaw );
    yaw = yaw < 0.0? 2*M_PI+yaw: yaw;

    if ( start_step_ == true ) {
        prev_pos_[0] = base_pose_msg->pose.pose.position.x; // x
        prev_pos_[1] = base_pose_msg->pose.pose.position.y; // y
        prev_pos_[2] = yaw;
        current_pos_[0] = base_pose_msg->pose.pose.position.x; // x
        current_pos_[1] = base_pose_msg->pose.pose.position.y; // y
        current_pos_[2] = yaw;
        robot_odom_[0] = 0.0;
        robot_odom_[1] = 0.0;
    }
    else {
        prev_pos_[0] = current_pos_[0];
        prev_pos_[1] = current_pos_[1];
        prev_pos_[2] = current_pos_[2];
        current_pos_[0] = base_pose_msg->pose.pose.position.x; // x
        current_pos_[1] = base_pose_msg->pose.pose.position.y; // y
        current_pos_[2] = yaw;
        robot_odom_[0] = sqrt( pow(current_pos_[0]-prev_pos_[0], 2)+
                               pow(current_pos_[1]-prev_pos_[1], 2) );
        robot_odom_[1] = atan2( sin(current_pos_[2]-prev_pos_[2]),
                                cos(current_pos_[2]-prev_pos_[2]));
    }

    {
        boost::mutex::scoped_lock lock( scan_mutex_ );
        scan_empty_ = false;
    }
    scan_cond_.notify_one();
}



/** main process function for localization */
void PFLocalization::process() {
    while ( !exit_flag_ ) {
        ROS_INFO_ONCE( "Start PF based localization" );
        {
            boost::mutex::scoped_lock lock( scan_mutex_ );
            while ( scan_empty_ )
                scan_cond_.wait( lock );
        }

        {
            if ( start_step_ == true ) {
                // init particles
                init_particles();
                start_step_ = false;
            }
            else {

                if ( (robot_odom_[0] != 0.0 || robot_odom_[1] != 0.0) ) {
                    ROS_INFO("Robot odom: distance = %f and orientation = %f", robot_odom_[0], robot_odom_[1]);
                    motion( robot_odom_[0], robot_odom_[1] );

                    for(int i = 0; i < particles_.size(); i++)
                    {
                        double xPoint = particles_[i].x;
                        double yPoint = particles_[i].y;
                        double yaw = particles_[i].o;
                        particles_[i].weight = particles_[i].weight*sense(0.5,xPoint,yPoint,yaw);//finding the numerator of bayes rule
                    }

                    double sum = 0;
                    //finding the normaliser constant
                    for(int i = 0; i < particles_.size(); i++)
                    {
                        sum += particles_[i].weight;
                    }
                    //checks if the normaliser constant is very small, if it is, then the particles will re-initalise, this is done
                    //to try and minimise the chances on the algorith picking an incorrect particle
                    if(sum <= 0.0001)
                    {
                        init_particles();
                    }

                    for(int i = 0; i < particles_.size(); i++)
                    {
                        particles_[i].weight = particles_[i].weight/sum;//normalising
                    }

                    //The following few lines are just there to show that the sum of probabilities equals to 1
                    sum = 0;
                    for(int i = 0; i < particles_.size(); i++)
                    {
                        sum += particles_[i].weight;
                    }

                    std::cout<<"sum: "<<sum<<std::endl;

                    //std::cin.get();


										//! add update code block here
										//! Notice:
										//!		1. what is the likelihood
										//!		2. how to update previous belief
										//!		3. how to do normalization of updated belief

                    calc_estimate();

                    if ( step_count_ == 5 ) {

                        resampling();
                        step_count_ = 0;
                    }
                    step_count_ ++;

                }

                // publish topics
                geometry_msgs::PoseStamped est_pos_msg;
                est_pos_msg.header.stamp = ros::Time::now();
                est_pos_msg.header.frame_id = "/map";
                tf::poseTFToMsg( tf::Pose(tf::createQuaternionFromYaw(esti_pos_[2]),
                                          tf::Vector3(esti_pos_[0], esti_pos_[1], 0.0)),
                                 est_pos_msg.pose);
                esti_pos_pub_.publish( est_pos_msg );

                geometry_msgs::PoseArray particles_msg;
                particles_msg.header.stamp = ros::Time::now();
                particles_msg.header.frame_id = "/map";
                particles_msg.poses.resize( particles_.size() );
                for ( int i = 0; i < (int)particles_.size(); ++ i ) {
                    Particle & p = particles_[i];
                    if ( p.x < map_width/2.0 && p.x > -map_width/2.0 &&
                         p.y < map_height/2.0 && p.y > -map_height/2.0)  {
                        tf::poseTFToMsg( tf::Pose(tf::createQuaternionFromYaw(particles_[i].o),
                                                  tf::Vector3( particles_[i].x, particles_[i].y, 0.0 )),
                                         particles_msg.poses[i]);
                    }
                }
                particles_pub_.publish( particles_msg );

            }

            scan_mutex_.lock();
            scan_empty_ = true;
            scan_mutex_.unlock();

        }
    }
}

/** particle filter init */
void PFLocalization::init_particles() {
    particles_.resize( n_particles_ );

    double part_dist = 1.0/n_particles_;
    double Wspread = map_width/2;
    double Hspread = map_height/2;

		//! add particles initialistion code here
		//! Notice:
		//!		1. x, y range
		//!   2. orientation range
        //! 	3. weight
    //uniformly distributes positions, probabilities and orientations of the particles
    for (int i = 0; i < particles_.size(); i++)
    {
        Particle part;

        part.weight = part_dist;

        part.o = uniform_sampling(0,(2*M_PI));


        part.x = uniform_sampling(-Wspread,Wspread);
        part.y = uniform_sampling(-Hspread,Hspread);


        particles_[i] = part;
    }

    //std::cin.get();




}

/** sense function */
double PFLocalization::sense(double sigma, double x, double y, double theta) {
    double error = 1.0;
    double step_rad = M_PI/(scan_data_.size()-1);
    // compute particle position in pixel coordinate
    int px, py;
    px = (x+map_width/2)*100.0;
    py = (map_height-(y+map_height/2))*100.0;
    cv::Point2d stpt(px, py);

    for ( int i = 0; i < (int)scan_data_.size(); ++ i ) {
        // get accurate ray distance
        double raydist = 0.0;
        double c_theta = theta-M_PI/2+step_rad*i;

        double PreA;


        if ( c_theta > M_PI*2 )
            c_theta -= M_PI*2;
        if ( c_theta < 0 )
            c_theta += M_PI*2;
        cv::Point2d tgpt;
        tgpt.x = stpt.x+1000*cos(2*M_PI-c_theta);
        tgpt.y = stpt.y+1000*sin(2*M_PI-c_theta);
        cv::Point2d obspt;
        cv::LineIterator it(map_img_, stpt, tgpt, 8);
        for ( int j = 0; j < it.count; ++ j, ++ it ){
            cv::Point2d pt = it.pos();
            if ( static_cast<int>(map_img_.at<uchar>(pt)) != 255 ||
                 pt.x == 0 || pt.x == map_img_.cols-1 ||
                 pt.y == 0 || pt.y == map_img_.rows-1 ) {
                obspt = it.pos();
                break;
            }
        }
				//! raydist is the scan data given particle's pose
				//! scan_data_[i] is the actually sensor data
        raydist = sqrt((obspt.x-stpt.x)/100.0*(obspt.x-stpt.x)/100.0+
                       (obspt.y-stpt.y)/100.0*(obspt.y-stpt.y)/100.0);
				//! compute the likelihood of each beam				
				//! put your code here


        PreA = 1/(sqrt(2*M_PI*pow(sigma,2)));

        error *= PreA*exp((-1*pow((raydist - scan_data_[i]),2))/(2*pow(sigma,2)));




    }

    //std::cin.get();
		//! return the final error
		//! Notice:
		//!		1. how to combine the likelihood of each beam together

    return error;
}

/** motion function */
void PFLocalization::motion(double dist, double ori) {
    // check dist negativity
    if ( dist < 0 ) {
        cout << "ERROR: distance < 0\n";
        exit(-1);
    }

    double noise;
    double angNoise;

    double NewAngle;

    for ( int i = 0; i < (int)particles_.size(); ++ i ) {

        noise = gaussian_sampling(0,dist_noise_);
        angNoise = gaussian_sampling(0,ori_noise_);

        particles_[i].x += (dist + noise)*cos(particles_[i].o);
        particles_[i].y += (dist + noise)*sin(particles_[i].o);

        NewAngle = (particles_[i].o + (ori + angNoise));

        if (NewAngle > 2*M_PI)
        {
            NewAngle = NewAngle - (2*M_PI);
        }



        particles_[i].o = NewAngle;

        //! add noise to motion
				//! put your code here
				//! Notice:
				//! 	1. sampling distribution
				//!  	2. range of updated orientation
    }

    //std::cin.get();
}

/** calculate estimate robot pose */
void PFLocalization::calc_estimate() {
    // reset estimated pose
    esti_pos_[0] = 0.0;
    esti_pos_[1] = 0.0;
    esti_pos_[2] = 0.0;

    double sumx = 0;
    double sumy = 0;
    double sino = 0;
    double coso = 0;

    //! compute estimate pose
		//! put your code here
		//! Notice:
		//! 	1. average, how?
		//!		2. again, range of x, y, orientation


    for (int i = 0; i < particles_.size(); i++)
    {
        sumx += particles_[i].x * particles_[i].weight;
        sumy += particles_[i].y * particles_[i].weight;

        sino += particles_[i].weight * sin(particles_[i].o);
        coso += particles_[i].weight * cos(particles_[i].o);
    }

    esti_pos_[0] = sumx;
    esti_pos_[1] = sumy;
    esti_pos_[2] = atan2(sino/n_particles_,coso/n_particles_);

    //std::cin.get();




		//! uncomment the following line if you want to see the estimated pose
    cout << "Estimated pose: " << esti_pos_[0] << ", " << esti_pos_[1] << ", " << esti_pos_[2] << endl;
}

/** resampling particles */
/*
 * What the code is essentially doing is picking out best fit particles, however, the way its set up it doesn't just pick the
 * highest weighted particles but instead gives a chance for the smaller particles to propagate aswell (it however does have
 * preference for larger weighted particles)
 *
 * So what you can say is that beta is constantly changing and we're also randomly selecting a particle from the list.
 * If that particles weight is larger than beta it gets picked. If not its weight is subtracted from beta, we pick another particle
 * and try again
 *
 * Since beta is defined using the highest possible weight, particles that have a large weight will be picked more often,
 * but particles with Lower weights still have a chance since we randomly pick particles
 *
 * Of course in this particular code we also ignore any particles whose co ordiantes move outside the bounds of the map (which
 * makes sense)
 *
 *
 */
void PFLocalization::resampling() {
    int ind = int(uniform_sampling(0., 1.)*double(particles_.size())); //initialising the first postion to test
    double beta = 0.0;
    double maxw = 0;
    for ( int i = 0; i < (int)particles_.size(); ++ i ) //This for loop locates the particle which has the highest weight
        if ( particles_[i].weight > maxw )
            maxw = particles_[i].weight;
    vector<Particle> n_particles;
    for ( int i = 0; i < (int)particles_.size(); ++ i ) {
        beta += uniform_sampling(0., 1.0)*2.0*maxw; //defining beta in relation to the particle with the highest weight
        // This will allow us to better scope which particles we compare
        bool found = false;
        while ( !found ) {
            beta -= particles_[ind].weight; //with every iteration we decrease the value of beta which increases the chance that
            //a particle will be picked (since every particle will have a weight which is equal to or less than the max)
            //it is also decreased on the first iteration to allow the first selected particle a more fair chance to be selected
            ind = int(fmod((ind+1), double(particles_.size())));//the next particle selected is based on the remainder (rounded to
            //nearest int) of the division between one position up and the total amount of particles
            if ( particles_[ind].x > map_width/2 || particles_[ind].x < -map_width/2 ||
                 particles_[ind].y > map_height/2 || particles_[ind].y < -map_height/2 ) //particles outside the bounds are ignored
                continue;
            if ( beta > particles_[ind].weight )//any particles whose weight is larger than the ever decreasing beta is selected
                continue;
            found = true;
            //when a particle is selected beta will reset and the whole process starts again (running as many times as there are
            //particles) of course, as a result larger weighted particles will be picked more often than the lower weighted ones
            //and since we dont remove any selected particle from the stack, we can pick the same particle multiple times
            //as long as its weight is less than beta

        }
        n_particles.push_back( particles_[ind] );
    }
    particles_ = n_particles;

    // add jitter
    for( int k = 0; k < (int)particles_.size(); ++ k) {
        particles_[k].x += gaussian_sampling(0., 0.02);
        particles_[k].y += gaussian_sampling(0., 0.02);
        particles_[k].o += fmod(gaussian_sampling(0., 0.05),2*M_PI);
    }
}


/** generate uniform sampling */
double PFLocalization::uniform_sampling( double min, double max ) {
    static boost::mt19937 rng( static_cast<unsigned> (time(0)) );
    boost::uniform_real<double> uni_dist(min, max);
    boost::variate_generator<boost::mt19937&, boost::uniform_real<double> > uni_gen(rng, uni_dist);
    return uni_gen();
}

/** generate gaussian  */
double PFLocalization::gaussian_sampling( double mean, double sigma ) {
    static boost::mt19937 rng( static_cast<unsigned> (time(0)) );
    boost::normal_distribution<double> norm_dist( mean, sigma );
    boost::variate_generator<boost::mt19937&, boost::normal_distribution<double> > norm_gen(rng, norm_dist);
    return norm_gen();
}

/** command callback */
void PFLocalization::cmd_callback(const geometry_msgs::TwistConstPtr &cmd_msg) {
    ROS_INFO( "Command received" );
}


/** PFLocalization constructor */
PFLocalization::PFLocalization(ros::NodeHandle &nh, int n_particles ) {
    nh_ = &nh;
    exit_flag_ = false;
    scan_empty_ = true;
    map_img_ = cv::imread( ros::package::getPath("pf_localization")+"/data/map.png", CV_LOAD_IMAGE_GRAYSCALE );
    start_step_ = true;

    step_count_ = 0;

    // init noise in motion
    dist_noise_ = 0.01;

    ori_noise_ = (3.0/180.0*M_PI); // in degree

    scan_data_.resize( nbeams );
    scan_topic_name_ = "/base_scan";
    pose_topic_name_ = "/base_pose_ground_truth";

    particles_pub_ = nh.advertise<geometry_msgs::PoseArray>("/particles", 10);

    // reset estimated pose
    esti_pos_[0] = 0.0;
    esti_pos_[1] = 0.0;
    esti_pos_[2] = 0.0;
    esti_pos_pub_  = nh.advertise<geometry_msgs::PoseStamped>("/esti_pose", 10);

    n_particles_ = n_particles;

    robot_laser_sub_.subscribe( nh, scan_topic_name_, 1 );
    robot_pose_sub_.subscribe( nh, pose_topic_name_, 1 );

    cmd_sub_ = nh.subscribe("/cmd_vel", 1, &PFLocalization::cmd_callback, this);

    robot_sync_.reset( new message_filters::Synchronizer<robot_policy>(robot_policy(100), robot_laser_sub_, robot_pose_sub_) );
    robot_sync_->registerCallback( boost::bind(&PFLocalization::robot_callback, this, _1, _2) );


    process_thread_ = boost::thread( boost::bind(&PFLocalization::process, this) );
}

int main( int argc, char ** argv ) {
    ros::init( argc, argv, "pf_localizaton" );
    ros::NodeHandle nh("~");
    PFLocalization pf(nh, 1000);
    ros::spin();
    return 0;
}
