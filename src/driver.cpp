#include <boost/program_options.hpp>
#include <boost/tokenizer.hpp>
#include "LNS.h"
#include "AnytimeBCBS.h"
#include "AnytimeEECBS.h"
#include "PIBT/pibt.h"
#include "UAVLifelong.h"


/* Main function */
int main(int argc, char** argv)
{
	namespace po = boost::program_options;
	// Declare the supported options.
	po::options_description desc("Allowed options");
	po::options_description uav_options("UAV extension options");
	desc.add_options()
		("help", "produce help message")

		// params for the input instance and experiment settings
		("map,m", po::value<string>()->required(), "input file for map")
		("agents,a", po::value<string>()->required(), "input file for agents")
		("agentNum,k", po::value<int>()->default_value(0), "number of agents")
        ("output,o", po::value<string>(), "output file name (no extension)")
        ("outputPaths", po::value<string>(), "output file for paths")
        ("uavConfig", po::value<string>(),
         "UAV-only configuration file (Safe-Radius and Lifelong options)")
        ("cutoffTime,t", po::value<double>()->default_value(7200), "cutoff time (seconds)")
		("screen,s", po::value<int>()->default_value(0),
		        "screen option (0: none; 1: LNS results; 2:LNS detailed results; 3: MAPF detailed results)")
		("stats", po::value<string>(), "output stats file")

		// solver
		("solver", po::value<string>()->default_value("LNS"), "solver (LNS, A-BCBS, A-EECBS)")
		("sipp", po::value<bool>()->default_value(true), "Use SIPP as the single-agent solver")
		("seed", po::value<int>()->default_value(0), "Random seed")

        // params for LNS
        ("initLNS", po::value<bool>()->default_value(true),
             "use LNS to find initial solutions if the initial sovler fails")
        ("neighborSize", po::value<int>()->default_value(8), "Size of the neighborhood")
        ("maxIterations", po::value<int>()->default_value(0), "maximum number of iterations")
        ("initAlgo", po::value<string>()->default_value("PP"),
                "MAPF algorithm for finding the initial solution (EECBS, PP, PPS, CBS, PIBT, winPIBT)")
        ("replanAlgo", po::value<string>()->default_value("PP"),
                "MAPF algorithm for replanning (EECBS, CBS, PP)")
        ("destoryStrategy", po::value<string>()->default_value("Adaptive"),
                "Heuristics for finding subgroups (Random, RandomWalk, Intersection, Adaptive)")
        ("pibtWindow", po::value<int>()->default_value(5),
             "window size for winPIBT")
        ("winPibtSoftmode", po::value<bool>()->default_value(true),
             "winPIBT soft mode")

         // params for initLNS
         ("initDestoryStrategy", po::value<string>()->default_value("Adaptive"),
          "Heuristics for finding subgroups (Target, Collision, Random, Adaptive)")
		;

	uav_options.add_options()
        ("uavConfigVersion", po::value<int>()->default_value(1),
         "UAV configuration schema version")
        ("safeRadius", po::value<int>()->default_value(0),
         "Safety radius in grid cells (0=classic MAPF; e.g. 4=20m at 5m/pixel)")

        // event-driven multi-leg UAV mode
        ("lifelong", po::value<bool>()->default_value(false),
         "Enable event-driven multi-leg UAV planning")
        ("pairedTasks", po::value<bool>()->default_value(false),
         "Key follow-up task draws by seed, aircraft and leg for paired experiments")
        ("simulationTime", po::value<int>()->default_value(2000),
         "Global timestep horizon in lifelong mode")
        ("legsPerAircraft", po::value<int>()->default_value(3),
         "Maximum number of generated flight legs per aircraft")
        ("initialReleaseMax", po::value<int>()->default_value(0),
         "Uniform initial release-time upper bound (lower bound is 0)")
        ("restMin", po::value<int>()->default_value(10),
         "Minimum rest time after arrival")
        ("restMax", po::value<int>()->default_value(30),
         "Maximum rest time after arrival")
        ("retryInterval", po::value<int>()->default_value(5),
         "Ground-delay timesteps between planning retries")
        ("maxRetries", po::value<int>()->default_value(100),
         "Maximum planning retries for one flight leg")
        ("maxWait", po::value<int>()->default_value(600),
         "Maximum departure delay before a flight is deferred")
        ("maxNoProgress", po::value<int>()->default_value(600),
         "Maximum time with READY flights and no active traffic or progress")
        ("planningTime", po::value<double>()->default_value(10),
         "Solver time limit for each lifelong admission attempt")
		;
	desc.add(uav_options);

	po::variables_map vm;
	try
	{
		// Store command-line values first. Boost.Program_options keeps explicit
		// non-default values, while values loaded from the UAV config replace
		// command-line defaults. This gives: defaults < UAV cfg < explicit CLI.
		po::store(po::parse_command_line(argc, argv, desc), vm);

		if (vm.count("help"))
		{
			cout << desc << endl;
			return 0;
		}

		if (vm.count("uavConfig"))
		{
			const string config_path = vm["uavConfig"].as<string>();
			std::ifstream config_stream(config_path);
			if (!config_stream)
			{
				cerr << "Cannot open UAV configuration file: " << config_path << endl;
				return 2;
			}
			// Only options declared in uav_options are legal in this file.
			// Upstream options such as map, agents and solver are rejected.
			po::store(po::parse_config_file(config_stream, uav_options, false), vm);
			cout << "Loaded UAV configuration: " << config_path << endl;
		}

		po::notify(vm);
	}
	catch (const po::error& error)
	{
		cerr << "Parameter error: " << error.what() << endl;
		return 2;
	}

    if (vm["uavConfigVersion"].as<int>() != 1)
    {
        cerr << "Unsupported uavConfigVersion: " << vm["uavConfigVersion"].as<int>() << endl;
        return 2;
    }

    PIBTPPS_option pipp_option;
    pipp_option.windowSize = vm["pibtWindow"].as<int>();
    pipp_option.winPIBTSoft = vm["winPibtSoftmode"].as<bool>();

	srand((int)time(0));

	Instance instance(vm["map"].as<string>(), vm["agents"].as<string>(),
		vm["agentNum"].as<int>());
    int safe_radius = vm["safeRadius"].as<int>();
    if (safe_radius > 0)
    {
        instance.precomputeSafeNeighbors(safe_radius);
    }
	double time_limit = vm["cutoffTime"].as<double>();
    int screen = vm["screen"].as<int>();
	srand(vm["seed"].as<int>());

    if (vm["lifelong"].as<bool>())
    {
        LifelongOptions options;
        options.horizon = vm["simulationTime"].as<int>();
        options.legs_per_aircraft = vm["legsPerAircraft"].as<int>();
        options.initial_release_max = vm["initialReleaseMax"].as<int>();
        options.rest_min = vm["restMin"].as<int>();
        options.rest_max = vm["restMax"].as<int>();
        options.retry_interval = vm["retryInterval"].as<int>();
        options.max_retries = vm["maxRetries"].as<int>();
        options.max_wait = vm["maxWait"].as<int>();
        options.max_no_progress = vm["maxNoProgress"].as<int>();
        options.planning_time = vm["planningTime"].as<double>();
        options.seed = vm["seed"].as<int>();
        options.paired_tasks = vm["pairedTasks"].as<bool>();
        options.screen = screen;
        options.use_sipp = vm["sipp"].as<bool>();
        options.neighbor_size = vm["neighborSize"].as<int>();
        options.use_init_lns = vm["initLNS"].as<bool>();
        options.init_algo = vm["initAlgo"].as<string>();
        options.replan_algo = vm["replanAlgo"].as<string>();
        options.destroy_strategy = vm["destoryStrategy"].as<string>();
        options.init_destroy_strategy = vm["initDestoryStrategy"].as<string>();
        options.pibt_option = pipp_option;

        if (options.rest_min < 0 || options.rest_max < options.rest_min ||
            options.initial_release_max < 0 || options.planning_time <= 0)
        {
            cerr << "Invalid lifelong time-range option" << endl;
            return 2;
        }

        UAVLifelongSystem system(instance, options);
        bool succ = system.run();
        if (succ && vm.count("outputPaths"))
            system.writePaths(vm["outputPaths"].as<string>());
        if (vm.count("output"))
            system.writeFlights(vm["output"].as<string>());
        return succ ? 0 : 3;
    }

	if (vm["solver"].as<string>() == "LNS")
    {
        LNS lns(instance, time_limit,
                vm["initAlgo"].as<string>(),
                vm["replanAlgo"].as<string>(),
                vm["destoryStrategy"].as<string>(),
                vm["neighborSize"].as<int>(),
                vm["maxIterations"].as<int>(),
                vm["initLNS"].as<bool>(),
                vm["initDestoryStrategy"].as<string>(),
                vm["sipp"].as<bool>(),
                screen, pipp_option);
        bool succ = lns.run();
        if (succ)
        {
            lns.validateSolution();
            if (vm.count("outputPaths"))
                lns.writePathsToFile(vm["outputPaths"].as<string>());
        }
        if (vm.count("output"))
            lns.writeResultToFile(vm["output"].as<string>());
        if (vm.count("stats"))
            lns.writeIterStatsToFile(vm["stats"].as<string>());
        // lns.writePathsToFile("path.txt");
    }
    else if (vm["solver"].as<string>() == "A-BCBS") // anytime BCBS(w, 1)
    {
        AnytimeBCBS bcbs(instance, time_limit, screen);
        bcbs.run();
        bcbs.validateSolution();
        if (vm.count("output"))
            bcbs.writeResultToFile(vm["output"].as<string>() + ".csv");
        if (vm.count("stats"))
            bcbs.writeIterStatsToFile(vm["stats"].as<string>());
    }
    else if (vm["solver"].as<string>() == "A-EECBS") // anytime EECBS
    {
        AnytimeEECBS eecbs(instance, time_limit, screen);
        eecbs.run();
        eecbs.validateSolution();
        if (vm.count("output"))
            eecbs.writeResultToFile(vm["output"].as<string>() + ".csv");
        if (vm.count("stats"))
            eecbs.writeIterStatsToFile(vm["stats"].as<string>());
    }
	else
    {
	    cerr << "Solver " << vm["solver"].as<string>() << " does not exist!" << endl;
	    exit(-1);
    }
	return 0;

}
