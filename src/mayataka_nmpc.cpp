#include <mayataka_nmpc.hpp>
class mayataka_nmpc
{
	private:		
		/*--------- ROS Communication Containers ------------- */
		ros::NodeHandle n;
			ros::Subscriber NodeShutDown_sub;
			std::string s_shutdown_topic;

		int dim_state_, dim_control_input_, dim_constraints_;

		// Define the model in NMPC.
		NMPCModel nmpc_model;
		// MultipleShootingCGMRES cgmres_solver;
		ContinuationGMRES cgmres_solver;
		Simulator cgmres_simulator;

	public:
		mayataka_nmpc()
		{
			if(ros::param::get("~shutdown_topic",s_shutdown_topic)){} else {s_shutdown_topic = "/kill";}

			ROS_INFO("mayataka_nmpc:: NodeShutDown_sub s_shutdown_topic.");
			NodeShutDown_sub 	= n.subscribe(s_shutdown_topic,		1, &mayataka_nmpc::nodeShutDown, 	this);

			// Define the solver of C/GMRES.
			
			
			double horizon_max_length = 1.0; // seconds
			double alpha = 1.0; // scale for time smoothing function
			double horizon_division_num = 50; 
			double difference_increment = 1e-6; // dt for continuation of Fu
			double zeta = 1000;  // "a positive real number"
			double dim_krylov = 3; // max_krylov = dim_krylov
			cgmres_solver.setSolver(nmpc_model, horizon_max_length, alpha, horizon_division_num, difference_increment, zeta, dim_krylov);

			// Set the initial state.
			Eigen::VectorXd initial_state(nmpc_model.dimState());
			initial_state = Eigen::VectorXd::Zero(nmpc_model.dimState());

			// Set the initial guess of the control input vector.
			Eigen::VectorXd initial_guess_control_input(nmpc_model.dimControlInput()+nmpc_model.dimConstraints());
			initial_guess_control_input = Eigen::VectorXd::Zero(nmpc_model.dimControlInput()+nmpc_model.dimConstraints());

			// Initialize the solution of the C/GMRES method.
			double initial_time = 0;
			double convergence_radius = 1.0e-06;
			int max_iteration = 50;
			cgmres_solver.initSolution(initial_time, initial_state, initial_guess_control_input, convergence_radius, max_iteration);

			ROS_INFO("mayataka_nmpc:: mayataka_nmpc started.");
			// Perform a numerical simulation.
			cgmres_simulator.initModel(nmpc_model);
			double start_time = 0; //seconds
			double end_time = 10; //seconds
			double sampling_period = 0.001; //seconds
			// std::string savefile_name = "example";
			cgmres_simulator.simulation(cgmres_solver, initial_state, start_time, end_time, sampling_period, "example");
		}

		void nodeShutDown(const std_msgs::EmptyConstPtr& msg)
		{
			ROS_INFO("mayataka_nmpc:: Shutdown requested..");
			ros::Duration(1.5).sleep();
			ros::shutdown();
		}
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "mayataka_nmpc");
	mayataka_nmpc model_;
	ros::spin();
	return 0;
}