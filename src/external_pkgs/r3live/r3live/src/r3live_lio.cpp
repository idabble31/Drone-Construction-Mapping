//r3live_lio_current_in_scanhub

/* This code is the implementation of our paper "R3LIVE: A Robust, Real-time, RGB-colored, 
LiDAR-Inertial-Visual tightly-coupled state Estimation and mapping package".
*/

#include "r3live.hpp"
#include <iostream>
#include <cmath>
#include <algorithm>

// --- FIX: Add this namespace so 'cout' and 'endl' work ---
using namespace std; 
// --------------------------------------------------------

void R3LIVE::imu_cbk( const sensor_msgs::Imu::ConstPtr &msg_in )
{
    sensor_msgs::Imu::Ptr msg( new sensor_msgs::Imu( *msg_in ) );
    double                timestamp = msg->header.stamp.toSec();
    g_camera_lidar_queue.imu_in( timestamp );
    mtx_buffer.lock();
    if ( timestamp < last_timestamp_imu )
    {
        ROS_ERROR( "imu loop back, clear buffer" );
        imu_buffer_lio.clear();
        imu_buffer_vio.clear();
        flg_reset = true;
    }

    last_timestamp_imu = timestamp;

    if ( g_camera_lidar_queue.m_if_acc_mul_G )
    {
        msg->linear_acceleration.x *= G_m_s2;
        msg->linear_acceleration.y *= G_m_s2;
        msg->linear_acceleration.z *= G_m_s2;
    }

    imu_buffer_lio.push_back( msg );
    imu_buffer_vio.push_back( msg );
    mtx_buffer.unlock();
    sig_buffer.notify_all();
}

void printf_field_name( sensor_msgs::PointCloud2::ConstPtr &msg )
{
    cout << "Input pointcloud field names: [" << msg->fields.size() << "]: ";
    for ( size_t i = 0; i < msg->fields.size(); i++ )
    {
        cout << msg->fields[ i ].name << ", ";
    }
    cout << endl;
}


bool R3LIVE::get_pointcloud_data_from_ros_message( sensor_msgs::PointCloud2::ConstPtr &msg, pcl::PointCloud< pcl::PointXYZINormal > &pcl_pc )
{
    pcl::PointCloud< pcl::PointXYZI > res_pc;
    scope_color( ANSI_COLOR_YELLOW_BOLD );

    if ( msg->fields.size() < 3 )
    {
        cout << "Get pointcloud data from ros messages fail!!!" << endl;
        scope_color( ANSI_COLOR_RED_BOLD );
        printf_field_name( msg );
        return false;
    }
    else
    {
        if ( ( msg->fields.size() == 8 ) && ( msg->fields[ 3 ].name == "intensity" ) &&
             ( msg->fields[ 4 ].name == "normal_x" ) ) // Input message type is pcl::PointXYZINormal
        {
            pcl::fromROSMsg( *msg, pcl_pc );
            return true;
        }
        else if ( ( msg->fields.size() == 4 ) && ( msg->fields[ 3 ].name == "rgb" ) )
        {
            double maximum_range = 5;
            get_ros_parameter< double >( m_ros_node_handle, "iros_range", maximum_range, 5 );
            pcl::PointCloud< pcl::PointXYZRGB > pcl_rgb_pc;
            pcl::fromROSMsg( *msg, pcl_rgb_pc );
            double lidar_point_time = msg->header.stamp.toSec();
            int    pt_count = 0;
            pcl_pc.resize( pcl_rgb_pc.points.size() );
            for ( int i = 0; i < pcl_rgb_pc.size(); i++ )
            {
                pcl::PointXYZINormal temp_pt;
                temp_pt.x = pcl_rgb_pc.points[ i ].x;
                temp_pt.y = pcl_rgb_pc.points[ i ].y;
                temp_pt.z = pcl_rgb_pc.points[ i ].z;
                double frame_dis = sqrt( temp_pt.x * temp_pt.x + temp_pt.y * temp_pt.y + temp_pt.z * temp_pt.z );
                if ( frame_dis > maximum_range )
                {
                    continue;
                }
                temp_pt.intensity = ( pcl_rgb_pc.points[ i ].r + pcl_rgb_pc.points[ i ].g + pcl_rgb_pc.points[ i ].b ) / 3.0;
                temp_pt.curvature = 0;
                pcl_pc.points[ pt_count ] = temp_pt;
                pt_count++;
            }
            pcl_pc.points.resize( pt_count );
            return true;
        }
        else 
        {
            cout << "Get pointcloud data from ros messages fail!!! ";
            scope_color( ANSI_COLOR_RED_BOLD );
            printf_field_name( msg );
            return false;
        }
    }
}

bool R3LIVE::sync_packages( MeasureGroup &meas )
{
    if ( lidar_buffer.empty() || imu_buffer_lio.empty() )
    {
        return false;
    }

    /*** push lidar frame ***/
    if ( !lidar_pushed )
    {
        meas.lidar.reset( new PointCloudXYZINormal() );
        if ( get_pointcloud_data_from_ros_message( lidar_buffer.front(), *( meas.lidar ) ) == false )
        {
            return false;
        }
        
        meas.lidar_beg_time = lidar_buffer.front()->header.stamp.toSec();
        lidar_end_time = meas.lidar_beg_time + meas.lidar->points.back().curvature / double( 1000 );
        meas.lidar_end_time = lidar_end_time;
        lidar_pushed = true;
    }

    if ( last_timestamp_imu < lidar_end_time )
    {
        return false;
    }

    /*** push imu data, and pop from imu buffer ***/
    double imu_time = imu_buffer_lio.front()->header.stamp.toSec();
    meas.imu.clear();
    while ( ( !imu_buffer_lio.empty() ) && ( imu_time < lidar_end_time ) )
    {
        imu_time = imu_buffer_lio.front()->header.stamp.toSec();
        if ( imu_time > lidar_end_time + 0.02 )
            break;
        meas.imu.push_back( imu_buffer_lio.front() );
        imu_buffer_lio.pop_front();
    }

    lidar_buffer.pop_front();
    lidar_pushed = false;
    return true;
}

// project lidar frame to world
void R3LIVE::pointBodyToWorld( PointType const *const pi, PointType *const po )
{
    Eigen::Vector3d p_body( pi->x, pi->y, pi->z );
    Eigen::Vector3d p_global( g_lio_state.rot_end * ( p_body + Lidar_offset_to_IMU ) + g_lio_state.pos_end );

    po->x = p_global( 0 );
    po->y = p_global( 1 );
    po->z = p_global( 2 );
    po->intensity = pi->intensity;
}

void R3LIVE::RGBpointBodyToWorld( PointType const *const pi, pcl::PointXYZI *const po )
{
    Eigen::Vector3d p_body( pi->x, pi->y, pi->z );
    Eigen::Vector3d p_global( g_lio_state.rot_end * ( p_body + Lidar_offset_to_IMU ) + g_lio_state.pos_end );

    po->x = p_global( 0 );
    po->y = p_global( 1 );
    po->z = p_global( 2 );
    po->intensity = pi->intensity;

    float intensity = pi->intensity;
    intensity = intensity - std::floor( intensity );

    int reflection_map = intensity * 10000;
}

int R3LIVE::get_cube_index( const int &i, const int &j, const int &k )
{
    return ( i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k );
}

bool R3LIVE::center_in_FOV( Eigen::Vector3f cube_p )
{
    Eigen::Vector3f dis_vec = g_lio_state.pos_end.cast< float >() - cube_p;
    float           squaredSide1 = dis_vec.transpose() * dis_vec;

    if ( squaredSide1 < 0.4 * cube_len * cube_len )
        return true;

    dis_vec = XAxisPoint_world.cast< float >() - cube_p;
    float squaredSide2 = dis_vec.transpose() * dis_vec;

    float ang_cos =
        fabs( squaredSide1 <= 3 ) ? 1.0 : ( LIDAR_SP_LEN * LIDAR_SP_LEN + squaredSide1 - squaredSide2 ) / ( 2 * LIDAR_SP_LEN * sqrt( squaredSide1 ) );

    return ( ( ang_cos > HALF_FOV_COS ) ? true : false );
}

bool R3LIVE::if_corner_in_FOV( Eigen::Vector3f cube_p )
{
    Eigen::Vector3f dis_vec = g_lio_state.pos_end.cast< float >() - cube_p;
    float           squaredSide1 = dis_vec.transpose() * dis_vec;
    dis_vec = XAxisPoint_world.cast< float >() - cube_p;
    float squaredSide2 = dis_vec.transpose() * dis_vec;
    float ang_cos =
        fabs( squaredSide1 <= 3 ) ? 1.0 : ( LIDAR_SP_LEN * LIDAR_SP_LEN + squaredSide1 - squaredSide2 ) / ( 2 * LIDAR_SP_LEN * sqrt( squaredSide1 ) );
    return ( ( ang_cos > HALF_FOV_COS ) ? true : false );
}

void R3LIVE::lasermap_fov_segment()
{
    laserCloudValidNum = 0;
    pointBodyToWorld( XAxisPoint_body, XAxisPoint_world );
    int centerCubeI = int( ( g_lio_state.pos_end( 0 ) + 0.5 * cube_len ) / cube_len ) + laserCloudCenWidth;
    int centerCubeJ = int( ( g_lio_state.pos_end( 1 ) + 0.5 * cube_len ) / cube_len ) + laserCloudCenHeight;
    int centerCubeK = int( ( g_lio_state.pos_end( 2 ) + 0.5 * cube_len ) / cube_len ) + laserCloudCenDepth;
    if ( g_lio_state.pos_end( 0 ) + 0.5 * cube_len < 0 )
        centerCubeI--;
    if ( g_lio_state.pos_end( 1 ) + 0.5 * cube_len < 0 )
        centerCubeJ--;
    if ( g_lio_state.pos_end( 2 ) + 0.5 * cube_len < 0 )
        centerCubeK--;
    bool last_inFOV_flag = 0;
    int  cube_index = 0;
    cub_needrm.clear();
    cub_needad.clear();
    T2[ time_log_counter ] = Measures.lidar_beg_time;
    double t_begin = omp_get_wtime();

    while ( centerCubeI < FOV_RANGE + 1 )
    {
        for ( int j = 0; j < laserCloudHeight; j++ )
        {
            for ( int k = 0; k < laserCloudDepth; k++ )
            {
                int i = laserCloudWidth - 1;

                PointCloudXYZINormal::Ptr laserCloudCubeSurfPointer = featsArray[ get_cube_index( i, j, k ) ];
                last_inFOV_flag = _last_inFOV[ cube_index ];

                for ( ; i >= 1; i-- )
                {
                    featsArray[ get_cube_index( i, j, k ) ] = featsArray[ get_cube_index( i - 1, j, k ) ];
                    _last_inFOV[ get_cube_index( i, j, k ) ] = _last_inFOV[ get_cube_index( i - 1, j, k ) ];
                }

                featsArray[ get_cube_index( i, j, k ) ] = laserCloudCubeSurfPointer;
                _last_inFOV[ get_cube_index( i, j, k ) ] = last_inFOV_flag;
                laserCloudCubeSurfPointer->clear();
            }
        }
        centerCubeI++;
        laserCloudCenWidth++;
    }

    while ( centerCubeI >= laserCloudWidth - ( FOV_RANGE + 1 ) )
    {
        for ( int j = 0; j < laserCloudHeight; j++ )
        {
            for ( int k = 0; k < laserCloudDepth; k++ )
            {
                int i = 0;

                PointCloudXYZINormal::Ptr laserCloudCubeSurfPointer = featsArray[ get_cube_index( i, j, k ) ];
                last_inFOV_flag = _last_inFOV[ cube_index ];

                for ( ; i >= 1; i-- )
                {
                    featsArray[ get_cube_index( i, j, k ) ] = featsArray[ get_cube_index( i + 1, j, k ) ];
                    _last_inFOV[ get_cube_index( i, j, k ) ] = _last_inFOV[ get_cube_index( i + 1, j, k ) ];
                }

                featsArray[ get_cube_index( i, j, k ) ] = laserCloudCubeSurfPointer;
                _last_inFOV[ get_cube_index( i, j, k ) ] = last_inFOV_flag;
                laserCloudCubeSurfPointer->clear();
            }
        }

        centerCubeI--;
        laserCloudCenWidth--;
    }

    while ( centerCubeJ < ( FOV_RANGE + 1 ) )
    {
        for ( int i = 0; i < laserCloudWidth; i++ )
        {
            for ( int k = 0; k < laserCloudDepth; k++ )
            {
                int j = laserCloudHeight - 1;

                PointCloudXYZINormal::Ptr laserCloudCubeSurfPointer = featsArray[ get_cube_index( i, j, k ) ];
                last_inFOV_flag = _last_inFOV[ cube_index ];

                for ( ; i >= 1; i-- )
                {
                    featsArray[ get_cube_index( i, j, k ) ] = featsArray[ get_cube_index( i, j - 1, k ) ];
                    _last_inFOV[ get_cube_index( i, j, k ) ] = _last_inFOV[ get_cube_index( i, j - 1, k ) ];
                }

                featsArray[ get_cube_index( i, j, k ) ] = laserCloudCubeSurfPointer;
                _last_inFOV[ get_cube_index( i, j, k ) ] = last_inFOV_flag;
                laserCloudCubeSurfPointer->clear();
            }
        }

        centerCubeJ++;
        laserCloudCenHeight++;
    }

    while ( centerCubeJ >= laserCloudHeight - ( FOV_RANGE + 1 ) )
    {
        for ( int i = 0; i < laserCloudWidth; i++ )
        {
            for ( int k = 0; k < laserCloudDepth; k++ )
            {
                int                       j = 0;
                PointCloudXYZINormal::Ptr laserCloudCubeSurfPointer = featsArray[ get_cube_index( i, j, k ) ];
                last_inFOV_flag = _last_inFOV[ cube_index ];

                for ( ; i >= 1; i-- )
                {
                    featsArray[ get_cube_index( i, j, k ) ] = featsArray[ get_cube_index( i, j + 1, k ) ];
                    _last_inFOV[ get_cube_index( i, j, k ) ] = _last_inFOV[ get_cube_index( i, j + 1, k ) ];
                }

                featsArray[ get_cube_index( i, j, k ) ] = laserCloudCubeSurfPointer;
                _last_inFOV[ get_cube_index( i, j, k ) ] = last_inFOV_flag;
                laserCloudCubeSurfPointer->clear();
            }
        }

        centerCubeJ--;
        laserCloudCenHeight--;
    }

    while ( centerCubeK < ( FOV_RANGE + 1 ) )
    {
        for ( int i = 0; i < laserCloudWidth; i++ )
        {
            for ( int j = 0; j < laserCloudHeight; j++ )
            {
                int                       k = laserCloudDepth - 1;
                PointCloudXYZINormal::Ptr laserCloudCubeSurfPointer = featsArray[ get_cube_index( i, j, k ) ];
                last_inFOV_flag = _last_inFOV[ cube_index ];

                for ( ; i >= 1; i-- )
                {
                    featsArray[ get_cube_index( i, j, k ) ] = featsArray[ get_cube_index( i, j, k - 1 ) ];
                    _last_inFOV[ get_cube_index( i, j, k ) ] = _last_inFOV[ get_cube_index( i, j, k - 1 ) ];
                }

                featsArray[ get_cube_index( i, j, k ) ] = laserCloudCubeSurfPointer;
                _last_inFOV[ get_cube_index( i, j, k ) ] = last_inFOV_flag;
                laserCloudCubeSurfPointer->clear();
            }
        }

        centerCubeK++;
        laserCloudCenDepth++;
    }

    while ( centerCubeK >= laserCloudDepth - ( FOV_RANGE + 1 ) )
    {
        for ( int i = 0; i < laserCloudWidth; i++ )
        {
            for ( int j = 0; j < laserCloudHeight; j++ )
            {
                int                       k = 0;
                PointCloudXYZINormal::Ptr laserCloudCubeSurfPointer = featsArray[ get_cube_index( i, j, k ) ];
                last_inFOV_flag = _last_inFOV[ cube_index ];

                for ( ; i >= 1; i-- )
                {
                    featsArray[ get_cube_index( i, j, k ) ] = featsArray[ get_cube_index( i, j, k + 1 ) ];
                    _last_inFOV[ get_cube_index( i, j, k ) ] = _last_inFOV[ get_cube_index( i, j, k + 1 ) ];
                }

                featsArray[ get_cube_index( i, j, k ) ] = laserCloudCubeSurfPointer;
                _last_inFOV[ get_cube_index( i, j, k ) ] = last_inFOV_flag;
                laserCloudCubeSurfPointer->clear();
            }
        }
        centerCubeK--;
        laserCloudCenDepth--;
    }

    cube_points_add->clear();
    featsFromMap->clear();
    memset( now_inFOV, 0, sizeof( now_inFOV ) );
    copy_time = omp_get_wtime() - t_begin;
    double fov_check_begin = omp_get_wtime();

    fov_check_time = omp_get_wtime() - fov_check_begin;

    double readd_begin = omp_get_wtime();
#ifdef USE_ikdtree
    if ( cub_needrm.size() > 0 )
        ikdtree.Delete_Point_Boxes( cub_needrm );
    delete_box_time = omp_get_wtime() - readd_begin;
    // s_plot4.push_back(omp_get_wtime() - t_begin); t_begin = omp_get_wtime();
    if ( cub_needad.size() > 0 )
        ikdtree.Add_Point_Boxes( cub_needad );
    readd_box_time = omp_get_wtime() - readd_begin - delete_box_time;
    // s_plot5.push_back(omp_get_wtime() - t_begin); t_begin = omp_get_wtime();
    if ( cube_points_add->points.size() > 0 )
        ikdtree.Add_Points( cube_points_add->points, true );
#endif
    readd_time = omp_get_wtime() - readd_begin - delete_box_time - readd_box_time;
}

// --- PATCH: THIS FUNCTION APPLIES THE ROTATION ---
void R3LIVE::feat_points_cbk( const sensor_msgs::PointCloud2::ConstPtr &msg_in )
{
    // 1. Copy the message 
    sensor_msgs::PointCloud2::Ptr msg( new sensor_msgs::PointCloud2( *msg_in ) );
    
    // 2. Apply Timestamp Correction 
    ros::Time correct_time = ros::Time( msg_in->header.stamp.toSec() - m_lidar_imu_time_delay );
    msg->header.stamp = correct_time;

    // --- ROTATION START ---
    if (!m_lidar_ext_R.isIdentity(1e-4) || !m_lidar_ext_t.isZero(1e-4)) 
    {
        // A. Convert ROS Msg -> PCL Cloud
        pcl::PointCloud<pcl::PointXYZINormal> temp_cloud;
        pcl::fromROSMsg(*msg, temp_cloud);

        // B. Apply Rotation to every point
        for (auto &p : temp_cloud.points) {
            // Check for valid points (skip NaNs) using standard library
            if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;

            Eigen::Vector3d pt_in(p.x, p.y, p.z);
            Eigen::Vector3d pt_out = m_lidar_ext_R * pt_in + m_lidar_ext_t;
            
            p.x = pt_out.x();
            p.y = pt_out.y();
            p.z = pt_out.z();
        }

        // C. Convert PCL Cloud -> Back to ROS Msg
        pcl::toROSMsg(temp_cloud, *msg);
        
        // D. Restore Header (toROSMsg overwrites it)
        msg->header.stamp = correct_time;
        msg->header.frame_id = msg_in->header.frame_id;
    }
    // --- ROTATION END ---

    if ( g_camera_lidar_queue.lidar_in( msg_in->header.stamp.toSec() + 0.1 ) == 0 )
    {
        return;
    }
    mtx_buffer.lock();
    
    if ( msg->header.stamp.toSec() < last_timestamp_lidar )
    {
        ROS_ERROR( "lidar loop back, clear buffer" );
        lidar_buffer.clear();
    }
    
    lidar_buffer.push_back( msg );
    last_timestamp_lidar = msg->header.stamp.toSec();
    mtx_buffer.unlock();
    sig_buffer.notify_all();
}

void R3LIVE::wait_render_thread_finish()
{
    if ( m_render_thread != nullptr )
    {
        m_render_thread->get(); // wait render thread to finish.
        // m_render_thread = nullptr;
    }
}
int R3LIVE::service_LIO_update()
{
    nav_msgs::Path path;
    path.header.stamp = ros::Time::now();
    path.header.frame_id = "world";
    /*** variables definition ***/
    Eigen::Matrix< double, DIM_OF_STATES, DIM_OF_STATES > G, H_T_H, I_STATE;
    G.setZero();
    H_T_H.setZero();
    I_STATE.setIdentity();

    cv::Mat matA1( 3, 3, CV_32F, cv::Scalar::all( 0 ) );
    cv::Mat matD1( 1, 3, CV_32F, cv::Scalar::all( 0 ) );
    cv::Mat matV1( 3, 3, CV_32F, cv::Scalar::all( 0 ) );
    cv::Mat matP( 6, 6, CV_32F, cv::Scalar::all( 0 ) );

    PointCloudXYZINormal::Ptr feats_undistort( new PointCloudXYZINormal() );
    PointCloudXYZINormal::Ptr feats_down( new PointCloudXYZINormal() );
    PointCloudXYZINormal::Ptr laserCloudOri( new PointCloudXYZINormal() );
    PointCloudXYZINormal::Ptr coeffSel( new PointCloudXYZINormal() );

    /*** variables initialize ***/
    FOV_DEG = fov_deg + 10;
    HALF_FOV_COS = std::cos( ( fov_deg + 10.0 ) * 0.5 * PI_M / 180.0 );

    for ( int i = 0; i < laserCloudNum; i++ )
    {
        featsArray[ i ].reset( new PointCloudXYZINormal() );
    }

    std::shared_ptr< ImuProcess > p_imu( new ImuProcess() );
    m_imu_process = p_imu;
    //------------------------------------------------------------------------------------------------------
    ros::Rate rate( 5000 );
    bool      status = ros::ok();
    g_camera_lidar_queue.m_liar_frame_buf = &lidar_buffer;
    set_initial_state_cov( g_lio_state );
    
    while ( ros::ok() )
    {
        if ( flg_exit )
            break;
        ros::spinOnce();
        std::this_thread::sleep_for( std::chrono::milliseconds( 1 ) );
        while ( g_camera_lidar_queue.if_lidar_can_process() == false )
        {
            ros::spinOnce();
            std::this_thread::yield();
            std::this_thread::sleep_for( std::chrono::milliseconds( THREAD_SLEEP_TIM ) );
        }
        std::unique_lock< std::mutex > lock( m_mutex_lio_process );
        if ( 1 )
        {
            Common_tools::Timer tim;
            if ( sync_packages( Measures ) == 0 )
            {
                continue;
            }
            int lidar_can_update = 1;
            g_lidar_star_tim = frame_first_pt_time;
            if ( flg_reset )
            {
                ROS_WARN( "reset when rosbag play back" );
                p_imu->Reset();
                flg_reset = false;
                continue;
            }
            g_LiDAR_frame_index++;
            tim.tic( "Preprocess" );
            double t0, t1, t2, t3, t4, t5, match_start, match_time, solve_start, solve_time, pca_time, svd_time;
            match_time = 0;
            kdtree_search_time = 0;
            solve_time = 0;
            pca_time = 0;
            svd_time = 0;
            t0 = omp_get_wtime();
            p_imu->Process( Measures, g_lio_state, feats_undistort );

            g_camera_lidar_queue.g_noise_cov_acc = p_imu->cov_acc;
            g_camera_lidar_queue.g_noise_cov_gyro = p_imu->cov_gyr;
            StatesGroup state_propagate( g_lio_state );

            if ( feats_undistort->empty() || ( feats_undistort == NULL ) )
            {
                frame_first_pt_time = Measures.lidar_beg_time;
                std::cout << "not ready for odometry" << std::endl;
                continue;
            }

            if ( ( Measures.lidar_beg_time - frame_first_pt_time ) < INIT_TIME )
            {
                flg_EKF_inited = false;
                std::cout << "||||||||||Initiallizing LiDAR||||||||||" << std::endl;
            }
            else
            {
                flg_EKF_inited = true;
            }
            
            /*** Compute the euler angle ***/
            Eigen::Vector3d euler_cur = RotMtoEuler( g_lio_state.rot_end );
            
            lasermap_fov_segment();
            downSizeFilterSurf.setInputCloud( feats_undistort );
            downSizeFilterSurf.filter( *feats_down );
            
            /*** initialize the map kdtree ***/
            if ( ( feats_down->points.size() > 1 ) && ( ikdtree.Root_Node == nullptr ) )
            {
                ikdtree.set_downsample_param( filter_size_map_min );
                ikdtree.Build( feats_down->points );
                flg_map_initialized = true;
                continue;
            }

            if ( ikdtree.Root_Node == nullptr )
            {
                flg_map_initialized = false;
                std::cout << "~~~~~~~ Initialize Map iKD-Tree Failed! ~~~~~~~" << std::endl;
                continue;
            }
            int featsFromMapNum = ikdtree.size();
            int feats_down_size = feats_down->points.size();
            
            /*** ICP and iterated Kalman filter update ***/
            PointCloudXYZINormal::Ptr coeffSel_tmpt( new PointCloudXYZINormal( *feats_down ) );
            PointCloudXYZINormal::Ptr feats_down_updated( new PointCloudXYZINormal( *feats_down ) );
            std::vector< double >      res_last( feats_down_size, 1000.0 );

            if ( featsFromMapNum >= 5 )
            {
                t1 = omp_get_wtime();

                if ( m_if_publish_feature_map )
                {
                    PointVector().swap( ikdtree.PCL_Storage );
                    ikdtree.flatten( ikdtree.Root_Node, ikdtree.PCL_Storage, NOT_RECORD );
                    featsFromMap->clear();
                    featsFromMap->points = ikdtree.PCL_Storage;

                    sensor_msgs::PointCloud2 laserCloudMap;
                    pcl::toROSMsg( *featsFromMap, laserCloudMap );
                    laserCloudMap.header.stamp = ros::Time::now();
                    laserCloudMap.header.frame_id = "world";
                    pubLaserCloudMap.publish( laserCloudMap );
                }

                std::vector< bool >                point_selected_surf( feats_down_size, true );
                std::vector< std::vector< int > > pointSearchInd_surf( feats_down_size );
                std::vector< PointVector >        Nearest_Points( feats_down_size );

                int  rematch_num = 0;
                bool rematch_en = 0;
                flg_EKF_converged = 0;
                deltaR = 0.0;
                deltaT = 0.0;
                t2 = omp_get_wtime();
                double maximum_pt_range = 0.0;
                
                for ( iterCount = 0; iterCount < NUM_MAX_ITERATIONS; iterCount++ )
                {
                    tim.tic( "Iter" );
                    match_start = omp_get_wtime();
                    laserCloudOri->clear();
                    coeffSel->clear();

                    /** closest surface search and residual computation **/
                    for ( int i = 0; i < feats_down_size; i += m_lio_update_point_step )
                    {
                        double     search_start = omp_get_wtime();
                        PointType &pointOri_tmpt = feats_down->points[ i ];
                        double     ori_pt_dis =
                            sqrt( pointOri_tmpt.x * pointOri_tmpt.x + pointOri_tmpt.y * pointOri_tmpt.y + pointOri_tmpt.z * pointOri_tmpt.z );
                        maximum_pt_range = std::max( ori_pt_dis, maximum_pt_range );
                        PointType &pointSel_tmpt = feats_down_updated->points[ i ];

                        pointBodyToWorld( &pointOri_tmpt, &pointSel_tmpt );
                        std::vector< float > pointSearchSqDis_surf;
                        auto &points_near = Nearest_Points[ i ];

                        if ( iterCount == 0 || rematch_en )
                        {
                            point_selected_surf[ i ] = true;
                            ikdtree.Nearest_Search( pointSel_tmpt, NUM_MATCH_POINTS, points_near, pointSearchSqDis_surf );
                            float max_distance = pointSearchSqDis_surf[ NUM_MATCH_POINTS - 1 ];
                            if ( max_distance > m_maximum_pt_kdtree_dis )
                            {
                                point_selected_surf[ i ] = false;
                            }
                        }

                        kdtree_search_time += omp_get_wtime() - search_start;
                        if ( point_selected_surf[ i ] == false )
                            continue;

                        double pca_start = omp_get_wtime();
                        cv::Mat matA0( NUM_MATCH_POINTS, 3, CV_32F, cv::Scalar::all( 0 ) );
                        cv::Mat matB0( NUM_MATCH_POINTS, 1, CV_32F, cv::Scalar::all( -1 ) );
                        cv::Mat matX0( NUM_MATCH_POINTS, 1, CV_32F, cv::Scalar::all( 0 ) );

                        for ( int j = 0; j < NUM_MATCH_POINTS; j++ )
                        {
                            matA0.at< float >( j, 0 ) = points_near[ j ].x;
                            matA0.at< float >( j, 1 ) = points_near[ j ].y;
                            matA0.at< float >( j, 2 ) = points_near[ j ].z;
                        }

                        cv::solve( matA0, matB0, matX0, cv::DECOMP_QR );

                        float pa = matX0.at< float >( 0, 0 );
                        float pb = matX0.at< float >( 1, 0 );
                        float pc = matX0.at< float >( 2, 0 );
                        float pd = 1;

                        float ps = sqrt( pa * pa + pb * pb + pc * pc );
                        pa /= ps;
                        pb /= ps;
                        pc /= ps;
                        pd /= ps;

                        bool planeValid = true;
                        for ( int j = 0; j < NUM_MATCH_POINTS; j++ )
                        {
                            if ( fabs( pa * points_near[ j ].x + pb * points_near[ j ].y + pc * points_near[ j ].z + pd ) >
                                 m_planar_check_dis )
                            {
                                if ( ori_pt_dis < maximum_pt_range * 0.90 || ( ori_pt_dis < m_long_rang_pt_dis ) )
                                {
                                    planeValid = false;
                                    point_selected_surf[ i ] = false;
                                    break;
                                }
                            }
                        }

                        if ( planeValid )
                        {
                            float pd2 = pa * pointSel_tmpt.x + pb * pointSel_tmpt.y + pc * pointSel_tmpt.z + pd;
                            double acc_distance = ( ori_pt_dis < m_long_rang_pt_dis ) ? m_maximum_res_dis : 1.0;
                            if ( pd2 < acc_distance )
                            {
                                point_selected_surf[ i ] = true;
                                coeffSel_tmpt->points[ i ].x = pa;
                                coeffSel_tmpt->points[ i ].y = pb;
                                coeffSel_tmpt->points[ i ].z = pc;
                                coeffSel_tmpt->points[ i ].intensity = pd2;
                                res_last[ i ] = std::abs( pd2 );
                            }
                            else
                            {
                                point_selected_surf[ i ] = false;
                            }
                        }
                        pca_time += omp_get_wtime() - pca_start;
                    }
                    
                    tim.tic( "Stack" );
                    double total_residual = 0.0;
                    laserCloudSelNum = 0;

                    for ( int i = 0; i < coeffSel_tmpt->points.size(); i++ )
                    {
                        if ( point_selected_surf[ i ] && ( res_last[ i ] <= 2.0 ) )
                        {
                            laserCloudOri->push_back( feats_down->points[ i ] );
                            coeffSel->push_back( coeffSel_tmpt->points[ i ] );
                            total_residual += res_last[ i ];
                            laserCloudSelNum++;
                        }
                    }
                    res_mean_last = total_residual / laserCloudSelNum;

                    match_time += omp_get_wtime() - match_start;
                    solve_start = omp_get_wtime();

                    /*** Computation of Measurement Jacobian matrix H and measurements vector ***/
                    Eigen::MatrixXd Hsub( laserCloudSelNum, 6 );
                    Eigen::VectorXd meas_vec( laserCloudSelNum );
                    Hsub.setZero();

                    for ( int i = 0; i < laserCloudSelNum; i++ )
                    {
                        const PointType &laser_p = laserCloudOri->points[ i ];
                        Eigen::Vector3d  point_this( laser_p.x, laser_p.y, laser_p.z );
                        point_this += Lidar_offset_to_IMU;
                        Eigen::Matrix3d point_crossmat;
                        point_crossmat << SKEW_SYM_MATRIX( point_this );

                        const PointType &norm_p = coeffSel->points[ i ];
                        Eigen::Vector3d  norm_vec( norm_p.x, norm_p.y, norm_p.z );

                        Eigen::Vector3d A( point_crossmat * g_lio_state.rot_end.transpose() * norm_vec );
                        Hsub.row( i ) << VEC_FROM_ARRAY( A ), norm_p.x, norm_p.y, norm_p.z;
                        meas_vec( i ) = -norm_p.intensity;
                    }

                    Eigen::Vector3d                               rot_add, t_add, v_add, bg_add, ba_add, g_add;
                    Eigen::Matrix< double, DIM_OF_STATES, 1 > solution;
                    Eigen::MatrixXd                               K( DIM_OF_STATES, laserCloudSelNum );

                    /*** Iterative Kalman Filter Update ***/
                    if ( !flg_EKF_inited )
                    {
                        cout << ANSI_COLOR_RED_BOLD << "Run EKF init" << ANSI_COLOR_RESET << endl;
                        set_initial_state_cov( g_lio_state );
                    }
                    else
                    {
                        auto &&Hsub_T = Hsub.transpose();
                        H_T_H.block< 6, 6 >( 0, 0 ) = Hsub_T * Hsub;
                        Eigen::Matrix< double, DIM_OF_STATES, DIM_OF_STATES > &&K_1 =
                            ( H_T_H + ( g_lio_state.cov / LASER_POINT_COV ).inverse() ).inverse();
                        K = K_1.block< DIM_OF_STATES, 6 >( 0, 0 ) * Hsub_T;

                        auto vec = state_propagate - g_lio_state;
                        solution = K * ( meas_vec - Hsub * vec.block< 6, 1 >( 0, 0 ) );

                        g_lio_state = state_propagate + solution;
                        print_dash_board();
                        
                        rot_add = solution.block< 3, 1 >( 0, 0 );
                        t_add = solution.block< 3, 1 >( 3, 0 );
                        flg_EKF_converged = false;
                        if ( ( ( rot_add.norm() * 57.3 - deltaR ) < 0.01 ) && ( ( t_add.norm() * 100 - deltaT ) < 0.015 ) )
                        {
                            flg_EKF_converged = true;
                        }

                        deltaR = rot_add.norm() * 57.3;
                        deltaT = t_add.norm() * 100;
                    }

                    g_lio_state.last_update_time = Measures.lidar_end_time;
                    euler_cur = RotMtoEuler( g_lio_state.rot_end );
                    dump_lio_state_to_log( m_lio_state_fp );

                    /*** Rematch Judgement ***/
                    rematch_en = false;
                    if ( flg_EKF_converged || ( ( rematch_num == 0 ) && ( iterCount == ( NUM_MAX_ITERATIONS - 2 ) ) ) )
                    {
                        rematch_en = true;
                        rematch_num++;
                    }

                    /*** Convergence Judgements and Covariance Update ***/
                    if ( rematch_num >= 2 || ( iterCount == NUM_MAX_ITERATIONS - 1 ) )
                    {
                        if ( flg_EKF_inited )
                        {
                            G.block< DIM_OF_STATES, 6 >( 0, 0 ) = K * Hsub;
                            g_lio_state.cov = ( I_STATE - G ) * g_lio_state.cov;
                            total_distance += ( g_lio_state.pos_end - position_last ).norm();
                            position_last = g_lio_state.pos_end;
                        }
                        solve_time += omp_get_wtime() - solve_start;
                        break;
                    }
                    solve_time += omp_get_wtime() - solve_start;
                } // End of iteration loop

                // ==========================================
                // DRIFT TRACKING CODE - After all iterations
                // ==========================================
                static Eigen::Vector3d initial_position;
                static bool position_initialized = false;
                static int drift_frame_count = 0;
                static double drift_start_time = 0;

                if (!position_initialized && flg_EKF_inited) {
                    initial_position = g_lio_state.pos_end;
                    position_initialized = true;
                    drift_start_time = Measures.lidar_end_time;
                    std::cout << "\n=== DRIFT TRACKING STARTED ===" << std::endl;
                    std::cout << "Initial position: " << initial_position.transpose() << std::endl;
                }

                if (position_initialized) {
                    drift_frame_count++;
                    
                    // Print every 50 frames
                    if (drift_frame_count % 50 == 0) {
                        Eigen::Vector3d total_drift = g_lio_state.pos_end - initial_position;
                        double elapsed_time = Measures.lidar_end_time - drift_start_time;
                        double drift_per_second = total_drift.norm() / elapsed_time;
                        
                        std::cout << "\n=== DRIFT ANALYSIS (Frame " << drift_frame_count << ") ===" << std::endl;
                        std::cout << "Time elapsed: " << elapsed_time << " seconds" << std::endl;
                        std::cout << "Current position: " << g_lio_state.pos_end.transpose() << std::endl;
                        std::cout << "Total drift (XYZ): " << total_drift.transpose() << " meters" << std::endl;
                        std::cout << "Total drift magnitude: " << total_drift.norm() << " meters" << std::endl;
                        std::cout << "Drift rate: " << drift_per_second << " m/s" << std::endl;
                        std::cout << "Drift per axis: X=" << fabs(total_drift(0)) 
                                    << " Y=" << fabs(total_drift(1)) 
                                    << " Z=" << fabs(total_drift(2)) << " meters" << std::endl;
                        std::cout << "Velocity: " << g_lio_state.vel_end.transpose() << std::endl;
                        std::cout << "Gyro bias: " << g_lio_state.bias_g.transpose() << std::endl;
                        std::cout << "Accel bias: " << g_lio_state.bias_a.transpose() << std::endl;
                        std::cout << "Map points: " << ikdtree.size() << std::endl;
                        std::cout << "Matched points: " << laserCloudSelNum << "/" << feats_down_size << std::endl;
                        std::cout << "Match ratio: " << (float)laserCloudSelNum/feats_down_size << std::endl;
                        std::cout << "Avg residual: " << res_mean_last << std::endl;
                        std::cout << "=================================\n" << std::endl;
                    }
                }

                t3 = omp_get_wtime();

                /*** add new frame points to map ikdtree ***/
                PointVector points_history;
                ikdtree.acquire_removed_points( points_history );

                memset( cube_updated, 0, sizeof( cube_updated ) );

                for ( int i = 0; i < points_history.size(); i++ )
                {
                    PointType &pointSel = points_history[ i ];

                    int cubeI = int( ( pointSel.x + 0.5 * cube_len ) / cube_len ) + laserCloudCenWidth;
                    int cubeJ = int( ( pointSel.y + 0.5 * cube_len ) / cube_len ) + laserCloudCenHeight;
                    int cubeK = int( ( pointSel.z + 0.5 * cube_len ) / cube_len ) + laserCloudCenDepth;

                    if ( pointSel.x + 0.5 * cube_len < 0 )
                        cubeI--;
                    if ( pointSel.y + 0.5 * cube_len < 0 )
                        cubeJ--;
                    if ( pointSel.z + 0.5 * cube_len < 0 )
                        cubeK--;

                    if ( cubeI >= 0 && cubeI < laserCloudWidth && cubeJ >= 0 && cubeJ < laserCloudHeight && cubeK >= 0 && cubeK < laserCloudDepth )
                    {
                        int cubeInd = cubeI + laserCloudWidth * cubeJ + laserCloudWidth * laserCloudHeight * cubeK;
                        featsArray[ cubeInd ]->push_back( pointSel );
                    }
                }

                for ( int i = 0; i < feats_down_size; i++ )
                {
                    pointBodyToWorld( &( feats_down->points[ i ] ), &( feats_down_updated->points[ i ] ) );
                }
                t4 = omp_get_wtime();
                
                ikdtree.Add_Points( feats_down_updated->points, true );
                
                kdtree_incremental_time = omp_get_wtime() - t4 + readd_time + readd_box_time + delete_box_time;
                t5 = omp_get_wtime();
            }

            // ==========================================
            // ZUPT (ZERO VELOCITY UPDATE) CODE
            // Place RIGHT BEFORE publishing
            // ==========================================
            static Eigen::Vector3d zupt_anchor_position;
            static bool zupt_active = false;
            static int zupt_stationary_count = 0;
            static int zupt_moving_count = 0;

            // Thresholds - adjust based on your sensor
            const double VELOCITY_THRESHOLD = 0.02;      // m/s
            const double POSITION_CHANGE_THRESHOLD = 0.02; // meters
            const int FRAMES_TO_CONFIRM_STATIONARY = 20;
            const int FRAMES_TO_CONFIRM_MOVING = 5;

            bool currently_stationary = (g_lio_state.vel_end.norm() < VELOCITY_THRESHOLD);

            static Eigen::Vector3d prev_position = g_lio_state.pos_end;
            double position_change = (g_lio_state.pos_end - prev_position).norm();
            prev_position = g_lio_state.pos_end;

            bool position_stable = (position_change < POSITION_CHANGE_THRESHOLD);

            if (currently_stationary && position_stable) {
                zupt_moving_count = 0;
                zupt_stationary_count++;
                
                if (zupt_stationary_count >= FRAMES_TO_CONFIRM_STATIONARY) {
                    if (!zupt_active) {
                        zupt_anchor_position = g_lio_state.pos_end;
                        zupt_active = true;
                        std::cout << "\n>>> ZUPT ACTIVATED - Position Locked <<<" << std::endl;
                        std::cout << "Anchor position: " << zupt_anchor_position.transpose() << std::endl;
                    }
                    
                    // Lock position to anchor
                    g_lio_state.pos_end = zupt_anchor_position;
                    
                    // Strongly damp velocity
                    g_lio_state.vel_end *= 0.5;
                }
            } else {
                zupt_stationary_count = 0;
                zupt_moving_count++;
                
                if (zupt_moving_count >= FRAMES_TO_CONFIRM_MOVING && zupt_active) {
                    zupt_active = false;
                    std::cout << "\n>>> ZUPT DEACTIVATED - System Moving <<<" << std::endl;
                }
            }

            if (zupt_active && g_LiDAR_frame_index % 10 == 0) {
                std::cout << "[ZUPT] Position locked at: " << zupt_anchor_position.transpose() 
                          << " (drift prevented)" << std::endl;
            }

            /******* Publish current frame points in world coordinates:  *******/
            laserCloudFullRes2->clear();
            *laserCloudFullRes2 = dense_map_en ? ( *feats_undistort ) : ( *feats_down );

            int laserCloudFullResNum = laserCloudFullRes2->points.size();

            pcl::PointXYZI temp_point;
            laserCloudFullResColor->clear();
            {
                for ( int i = 0; i < laserCloudFullResNum; i++ )
                {
                    RGBpointBodyToWorld( &laserCloudFullRes2->points[ i ], &temp_point );
                    laserCloudFullResColor->push_back( temp_point );
                }
                sensor_msgs::PointCloud2 laserCloudFullRes3;
                pcl::toROSMsg( *laserCloudFullResColor, laserCloudFullRes3 );
                laserCloudFullRes3.header.stamp.fromSec( Measures.lidar_end_time );
                laserCloudFullRes3.header.frame_id = "world";
                pubLaserCloudFullRes.publish( laserCloudFullRes3 );
            }

            if ( g_camera_lidar_queue.m_if_have_camera_data || (g_LiDAR_frame_index < 100) )
            {
                static std::vector< double > stastic_cost_time;
                Common_tools::Timer          tim;
                wait_render_thread_finish();
                if ( m_if_record_mvs )
                {
                    std::vector< std::shared_ptr< RGB_pts > > pts_last_hitted;
                    pts_last_hitted.reserve( 1e6 );
                    m_number_of_new_visited_voxel = m_map_rgb_pts.append_points_to_global_map(
                        *laserCloudFullResColor, Measures.lidar_end_time - g_camera_lidar_queue.m_first_imu_time, nullptr,
                        m_append_global_map_point_step );
                }
                stastic_cost_time.push_back( tim.toc( " ", 0 ) );
            }
            
            if(0) // Uncomment this code scope to enable the publish of effective points.
            {
                /******* Publish effective points *******/
                laserCloudFullResColor->clear();
                pcl::PointXYZI temp_point;
                for ( int i = 0; i < laserCloudSelNum; i++ )
                {
                    RGBpointBodyToWorld( &laserCloudOri->points[ i ], &temp_point );
                    laserCloudFullResColor->push_back( temp_point );
                }
                sensor_msgs::PointCloud2 laserCloudFullRes3;
                pcl::toROSMsg( *laserCloudFullResColor, laserCloudFullRes3 );
                laserCloudFullRes3.header.stamp.fromSec( Measures.lidar_end_time );
                laserCloudFullRes3.header.frame_id = "world";
                pubLaserCloudEffect.publish( laserCloudFullRes3 );
            }

            /******* Publish Maps:  *******/
            sensor_msgs::PointCloud2 laserCloudMap;
            pcl::toROSMsg( *featsFromMap, laserCloudMap );
            laserCloudMap.header.stamp.fromSec( Measures.lidar_end_time );
            laserCloudMap.header.frame_id = "world";
            pubLaserCloudMap.publish( laserCloudMap );

            /******* Publish Odometry ******/
            geometry_msgs::Quaternion geoQuat = tf::createQuaternionMsgFromRollPitchYaw( euler_cur( 0 ), euler_cur( 1 ), euler_cur( 2 ) );
            odomAftMapped.header.frame_id = "world";
            odomAftMapped.child_frame_id = "/aft_mapped";
            odomAftMapped.header.stamp = ros::Time::now();
            odomAftMapped.pose.pose.orientation.x = geoQuat.x;
            odomAftMapped.pose.pose.orientation.y = geoQuat.y;
            odomAftMapped.pose.pose.orientation.z = geoQuat.z;
            odomAftMapped.pose.pose.orientation.w = geoQuat.w;
            odomAftMapped.pose.pose.position.x = g_lio_state.pos_end( 0 );
            odomAftMapped.pose.pose.position.y = g_lio_state.pos_end( 1 );
            odomAftMapped.pose.pose.position.z = g_lio_state.pos_end( 2 );

            pubOdomAftMapped.publish( odomAftMapped );

            static tf::TransformBroadcaster br;
            tf::Transform                   transform;
            tf::Quaternion                  q;
            transform.setOrigin(
                tf::Vector3( odomAftMapped.pose.pose.position.x, odomAftMapped.pose.pose.position.y, odomAftMapped.pose.pose.position.z ) );
            q.setW( odomAftMapped.pose.pose.orientation.w );
            q.setX( odomAftMapped.pose.pose.orientation.x );
            q.setY( odomAftMapped.pose.pose.orientation.y );
            q.setZ( odomAftMapped.pose.pose.orientation.z );
            transform.setRotation( q );
            br.sendTransform( tf::StampedTransform( transform, ros::Time().fromSec( Measures.lidar_end_time ), "world", "/aft_mapped" ) );

            msg_body_pose.header.stamp = ros::Time::now();
            msg_body_pose.header.frame_id = "/camera_odom_frame";
            msg_body_pose.pose.position.x = g_lio_state.pos_end( 0 );
            msg_body_pose.pose.position.y = g_lio_state.pos_end( 1 );
            msg_body_pose.pose.position.z = g_lio_state.pos_end( 2 );
            msg_body_pose.pose.orientation.x = geoQuat.x;
            msg_body_pose.pose.orientation.y = geoQuat.y;
            msg_body_pose.pose.orientation.z = geoQuat.z;
            msg_body_pose.pose.orientation.w = geoQuat.w;

            /******* Publish Path ********/
            msg_body_pose.header.frame_id = "world";
            if ( frame_num > 10 )
            {
                path.poses.push_back( msg_body_pose );
            }
            pubPath.publish( path );
            
            /*** save debug variables ***/
            frame_num++;
            aver_time_consu = aver_time_consu * ( frame_num - 1 ) / frame_num + ( t5 - t0 ) / frame_num;
            T1[ time_log_counter ] = Measures.lidar_beg_time;
            s_plot[ time_log_counter ] = aver_time_consu;
            s_plot2[ time_log_counter ] = kdtree_incremental_time;
            s_plot3[ time_log_counter ] = kdtree_search_time;
            s_plot4[ time_log_counter ] = fov_check_time;
            s_plot5[ time_log_counter ] = t5 - t0;
            s_plot6[ time_log_counter ] = readd_box_time;
            time_log_counter++;
            fprintf( m_lio_costtime_fp, "%.5f %.5f\r\n", g_lio_state.last_update_time - g_camera_lidar_queue.m_first_imu_time, t5 - t0 );
            fflush( m_lio_costtime_fp );
        }
        status = ros::ok();
        rate.sleep();
    }
    return 0;
}