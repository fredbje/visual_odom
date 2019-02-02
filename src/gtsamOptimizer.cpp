#define _USE_MATH_DEFINES
#include <cmath> // Can axess pi as M_PI
#include <fstream>
#include <iomanip> // setprecision

#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/NonlinearEquality.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>

#include <gtsam/slam/StereoFactor.h>
#include <gtsam/slam/ProjectionFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/PoseTranslationPrior.h>
#include <gtsam/slam/PoseRotationPrior.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/slam/dataset.h> // For save and load functions

#include "vertigo/betweenFactorSwitchable.h"
#include "vertigo/switchVariableLinear.h"
#include "vertigo/gpsfactorswitchable.h"

#include "gtsamOptimizer.h"
#include "utils.h"

GtsamOptimizer::GtsamOptimizer()
{
}

GtsamOptimizer::~GtsamOptimizer() {
    LOG(INFO) << "GtsamOptimizer destructor called.";
}

void GtsamOptimizer::addPose(const gtsam::Pose3& estimate, const unsigned int& id, const double& timestamp)
{
    Graph& graph = graphs_[currentGraphId_];
    graph.newValues_.insert(gtsam::Symbol('x', id), estimate);
    poseIds_.push_back(id);
    graph.poseIds_.push_back(id);
    timestamps_.push_back(id);
    graph.timestamps_.push_back(timestamp);
    if(!graph.firstPoseInitialized_)
    {
        gtsam::noiseModel::Diagonal::shared_ptr priorPoseNoise = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << gtsam::Vector3(0.2, 0.2, 0.2), gtsam::Vector3(1, 1, 1)).finished()); // Assuming 0.2 rad in roll, pitch, yaw
        graph.newFactors_.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(gtsam::Symbol('x', id), estimate, priorPoseNoise);
        graph.firstPoseInitialized_ = true;
    }
}

void GtsamOptimizer::addPose(const gtsam::Pose3& estimate, const unsigned int& id, const double& timestamp, const oxts& navdata)
{
    addPose(estimate, id, timestamp);
    addGpsPrior(id, navdata);
}

bool GtsamOptimizer::isGpsBufferDiverse(const std::vector<std::pair<unsigned int, gtsam::Point3>>& gpsMeasurementBuffer)
{
    double minEast, maxEast, minNorth, maxNorth;
    bool firstIteration = true;
    for( const auto& measurement : gpsMeasurementBuffer )
    {
        double east = measurement.second.x();
        double north = measurement.second.y();

        if( firstIteration )
        {
            minEast = east;
            maxEast = east;
            minNorth = north;
            maxNorth = north;
            firstIteration = false;
            continue;
        }

        if(east < minEast)
            minEast = east;
        else if(east > maxEast)
            maxEast = east;

        if(north < minNorth)
            minNorth = north;
        else if(north > maxNorth)
            maxNorth = north;
    }
    return maxEast - minEast > 30.0 && maxNorth - minNorth > 30.0;
}

void GtsamOptimizer::insertGpsBufferInGraph(Graph& graph, unsigned int& switchIdGps)
{
    for( const auto& measurement : graph.gpsMeasurementBuffer_ )
    {
        if( useSwitchableGpsConstraints_ )
        {
            double switchPrior = 1.0;
            graph.newValues_.insert(gtsam::Symbol('g', switchIdGps), vertigo::SwitchVariableLinear(switchPrior));
            graph.newFactors_.add(gtsam::PriorFactor<vertigo::SwitchVariableLinear>(gtsam::Symbol('g', switchIdGps), vertigo::SwitchVariableLinear(switchPrior), switchPriorNoise_));
            graph.newFactors_.emplace_shared<vertigo::GpsFactorSwitchableLinear>(gtsam::Symbol('x', measurement.first), gtsam::Symbol('g', switchIdGps++), measurement.second, gpsNoise_);

        }
        else
        {
            gtsam::GPSFactor gpsFactor(gtsam::Symbol('x', measurement.first), measurement.second, gpsNoise_);
            graph.newFactors_.emplace_shared<gtsam::GPSFactor>(gpsFactor);
        }
    }
    graph.gpsMeasurementBuffer_.clear();
}

void GtsamOptimizer::addGpsPrior(const unsigned int& id, const oxts& navdata)
{
    static unsigned int switchIdGps = 0;

    if(!localOriginSet_)
    {
        enuProjection_.Reset(navdata.lat, navdata.lon, navdata.alt);
        localOriginSet_ = true;
        LOG(INFO) << "Initial global coordinates: " << std::setprecision(14) << navdata.lat << ", " << navdata.lon << ", " << navdata.alt;
    }

    static unsigned int count = 0;
    if (count++ % 10 != 0)
        return;

    double e, n, u;
    enuProjection_.Forward(navdata.lat, navdata.lon, navdata.alt, e, n, u);
    gtsam::Point3 enu(e, n, u);

    Graph& currentGraph = graphs_[currentGraphId_];
    if(!currentGraph.trajectoryInitializedInGlobalFrame_)
    {
        currentGraph.gpsMeasurementBuffer_.push_back(std::make_pair(id, enu));

        if(isGpsBufferDiverse(currentGraph.gpsMeasurementBuffer_))
        {
            currentGraph.factorsToRemove_.push_back(0);
            insertGpsBufferInGraph(currentGraph, switchIdGps);
            currentGraph.trajectoryInitializedInGlobalFrame_ = true;
        }
    }
    else
    {
        //if (count++ % 100 == 0)
        //    enu = enu + gtsam::Point3(30, 0, 0);

        if( useSwitchableGpsConstraints_ )
        {
            double switchPrior = 1.0;

            currentGraph.newValues_.insert(gtsam::Symbol('g', switchIdGps), vertigo::SwitchVariableLinear(switchPrior));
            currentGraph.newFactors_.add(gtsam::PriorFactor<vertigo::SwitchVariableLinear>(gtsam::Symbol('g', switchIdGps), vertigo::SwitchVariableLinear(switchPrior), switchPriorNoise_));
            currentGraph.newFactors_.emplace_shared<vertigo::GpsFactorSwitchableLinear>(gtsam::Symbol('x', id), gtsam::Symbol('g', switchIdGps++), enu, gpsNoise_);

        }
        else
        {
            gtsam::GPSFactor gpsFactor(gtsam::Symbol('x', id), enu, gpsNoise_);
            currentGraph.newFactors_.emplace_shared<gtsam::GPSFactor>(gpsFactor);
        }
    }
}

unsigned int GtsamOptimizer::getGraphIdFromFrameId(unsigned int frameId)
{
    for(unsigned int graphId = 0; graphId < graphs_.size(); graphId++)
    {
        if(graphs_[graphId].iSAM2_.valueExists(gtsam::Symbol('x', frameId)))
        {
            return graphId;
        }
    }
    LOG(ERROR) << "Could not find frameId " << frameId << " in any graph.";
    abort();
}

void GtsamOptimizer::addRelativePoseConstraint(const gtsam::Pose3& deltaT, unsigned int idFrom, unsigned int idTo, bool isLoopClosureConstraint)
{
    Graph& currentGraph = graphs_[currentGraphId_];
    if(isLoopClosureConstraint)
    {
        if( useSwitchableLoopConstraints_ )
        {
            double switchPrior = 1.0;
            static unsigned int switchId = 0;
            currentGraph.newValues_.insert(gtsam::Symbol('s', switchId), vertigo::SwitchVariableLinear(switchPrior));
            currentGraph.newFactors_.add(gtsam::PriorFactor<vertigo::SwitchVariableLinear>(gtsam::Symbol('s', switchId), vertigo::SwitchVariableLinear(switchPrior), switchPriorNoise_));
            currentGraph.newFactors_.emplace_shared<vertigo::BetweenFactorSwitchableLinear<gtsam::Pose3>>(gtsam::Symbol('x', idFrom), gtsam::Symbol('x', idTo), gtsam::Symbol('s', switchId++), deltaT, loopClosureNoise_);
        }
        else
        {
            currentGraph.newFactors_.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(gtsam::Symbol('x', idFrom), gtsam::Symbol('x', idTo), deltaT, loopClosureNoise_);
        }

        if( graphs_.size() > 1)
        {
            unsigned int graphIdFrom = getGraphIdFromFrameId(idFrom);
            unsigned int graphIdTo = getGraphIdFromFrameId(idTo);
            if(graphIdFrom != graphIdTo)
            {
                unsigned int graphIdKeep, graphIdThrow;
                if(graphs_[graphIdFrom].iSAM2_.size() > graphs_[graphIdTo].iSAM2_.size())
                {
                    graphIdKeep = graphIdFrom;
                    graphIdThrow = graphIdTo;
                }
                else
                {
                    graphIdKeep = graphIdTo;
                    graphIdThrow = graphIdFrom;
                }

                Graph& graphToKeep = graphs_[graphIdKeep];
                Graph& graphToThrow = graphs_[graphIdThrow];

                graphToKeep.newFactors_ += graphToThrow.newFactors_;
                if(graphToThrow.trajectoryInitializedInGlobalFrame_)
                {
                    graphToKeep.newFactors_ += graphToThrow.iSAM2_.getFactorsUnsafe();
                }
                else
                {
                    graphToKeep.newFactors_ += gtsam::NonlinearFactorGraph(graphToThrow.iSAM2_.getFactorsUnsafe().begin()+1, graphToThrow.iSAM2_.getFactorsUnsafe().end());
                }

                graphToKeep.newValues_.insert(graphToThrow.newValues_);
                graphToKeep.newValues_.insert(graphToThrow.currentEstimate_);
                graphs_.erase(graphs_.begin()+graphIdThrow);
                if(graphIdThrow < graphIdKeep)
                {
                    graphIdKeep--;
                }
                currentGraphId_ = graphIdKeep;
            }
        }
    }
    else
    {
        graphs_[currentGraphId_].newFactors_.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(gtsam::Symbol('x', idFrom), gtsam::Symbol('x', idTo), deltaT, odometryNoise_);
    }
}

void GtsamOptimizer::optimize()
{
    Graph& graph = graphs_[currentGraphId_];
    graph.iSAM2_.update(graph.newFactors_, graph.newValues_, graph.factorsToRemove_);
    graph.factorsToRemove_.clear();
    graph.currentEstimate_ = graph.iSAM2_.calculateEstimate();
    graph.newFactors_.resize(0);
    graph.newValues_.clear();
    LOG(DEBUG) << "Number of graphs: " << graphs_.size();
}

std::vector<gtsam::Pose3> GtsamOptimizer::getCurrentTrajectoryEstimate()
{
    std::vector<gtsam::Pose3> poses;
    for(const auto& poseId : poseIds_)
    {
        if(graphs_.size() == 1)
        {
            poses.push_back(graphs_[0].currentEstimate_.at<gtsam::Pose3>(gtsam::Symbol('x', poseId)));
        }
        else
        {
            bool found = false;
            for(const auto& graph : graphs_)
            {
                if(graph.currentEstimate_.exists(gtsam::Symbol('x', poseId)))
                {
                    poses.push_back(graph.currentEstimate_.at<gtsam::Pose3>(gtsam::Symbol('x', poseId)));
                    found = true;
                }
            }
            if(!found)
            {
                LOG(ERROR) << "Could not find frame " << poseId << " in current values.";
                abort();
            }
        }
    }
    return poses;
}

gtsam::Pose3 GtsamOptimizer::getCurrentPoseEstimate(unsigned int frameId)
{
    if(graphs_.size() == 1)
    {
        return graphs_[0].currentEstimate_.at<gtsam::Pose3>(gtsam::Symbol('x', frameId));
    }
    else
    {
        for(const auto& graph : graphs_)
        {
            if(graph.currentEstimate_.exists(gtsam::Symbol('x', frameId)))
            {
                return graph.currentEstimate_.at<gtsam::Pose3>(gtsam::Symbol('x', frameId));
            }
        }
        LOG(ERROR) << "frame: " << frameId << "does not exist in values";
        abort();
    }
}

void GtsamOptimizer::saveTrajectoryLatLon(const std::string& outputFile) {
    LOG(INFO) << "Saving GPS track to file...";
    std::ofstream f;
    f.open(outputFile);

    for(const auto& poseId : poseIds_) {
        if(graphs_.size() == 1)
        {
            gtsam::Pose3 tempPose = graphs_[0].currentEstimate_.at<gtsam::Pose3>(gtsam::Symbol('x', poseId));
            double lat, lon, h;
            enuProjection_.Reverse(tempPose.x(), tempPose.y(), tempPose.z(), lat, lon, h);
            f << std::setprecision(14) << lat << " " << lon << "\n";
        }
        else
        {
            bool found = false;
            for(unsigned int graphId = 0; graphId < graphs_.size(); graphId++)
            {
                if(graphs_[graphId].currentEstimate_.exists(gtsam::Symbol('x', poseId)))
                {
                    gtsam::Pose3 tempPose = graphs_[graphId].currentEstimate_.at<gtsam::Pose3>(gtsam::Symbol('x', poseId));
                    double lat, lon, h;
                    enuProjection_.Reverse(tempPose.x(), tempPose.y(), tempPose.z(), lat, lon, h);
                    f << std::setprecision(14) << lat << " " << lon << "\n";
                    found = true;
                }
            }
            if(!found)
            {
                LOG(ERROR) << "Could not find frameid " << poseId << " in current values";
                abort();
            }
        }
    }
}

void GtsamOptimizer::setTrackLost()
{
    currentGraphId_ = static_cast<unsigned int>(graphs_.size());
    graphs_.emplace_back(gtsam::ISAM2(iSAM2Params_));
}

void GtsamOptimizer::saveGraphAndValues(const std::string& outputFile)
{
    if(graphs_.size() == 1)
    {
        gtsam::writeG2o(graphs_.back().iSAM2_.getFactorsUnsafe(), graphs_.back().currentEstimate_, outputFile);
    }

    for( unsigned int graphId = 0; graphId < graphs_.size(); graphId++)
    {
        std::string outputFileWithGraphId = outputFile + std::to_string(graphId);
        gtsam::writeG2o(graphs_[graphId].iSAM2_.getFactorsUnsafe(), graphs_[graphId].currentEstimate_, outputFileWithGraphId);
    }
}

void GtsamOptimizer::loadGraphAndValues(const std::string& inputFile)
{
    gtsam::NonlinearFactorGraph::shared_ptr actualGraph;
    gtsam::Values::shared_ptr actualValues;
    bool is3D = true;
    boost::tie(actualGraph, actualValues) = gtsam::readG2o(inputFile, is3D);
    // TODO actually use loaded graph and values
}

void GtsamOptimizer::saveSettings(const std::string& settingsFile)
{
    std::fstream f;
    f.open(settingsFile, std::ios_base::app);
    if(f.is_open())
    {
        f << "////////////////// GtsamOptimizer Settings //////////////////" << std::endl;
        f << "useSwitchableLoopConstraints: " << (useSwitchableLoopConstraints_ ? "true" : "false") << std::endl;
#ifdef USE_GN_PARAMS
        f << "OptimizationParams: GaussNewton" << std::endl;
        f << "wildfireThreshold: " << wildfireThreshold_ << std::endl;
#else
        f << "OptimizationParams: DogLeg" << std::endl;
        f << "initialDelta: " << initialDelta_ << std::endl;
        f << "wildfireThreshold: " << wildfireThreshold_;
        f << "adaptationMode: ";
        switch (adaptationMode_) {
        case gtsam::DoglegOptimizerImpl::SEARCH_EACH_ITERATION:
            f << "SEARCH_EACH_ITERATION";
            break;
        case gtsam::DoglegOptimizerImpl::SEARCH_REDUCE_ONLY:
            f << "SEARCH_REDUCE_ONLY";
            break;
        case gtsam::DoglegOptimizerImpl::ONE_STEP_PER_ITERATION:
            f << "ONE_STEP_PER_ITERATION";
        }
        f << std::endl;
        f << "verbose: " << (verbose_ ? "true" : "false") << std::endl;
#endif
        //f << "RelinearizationThreshold: " << relinearizationThreshold_ << std::endl;
        f << "relinearizeSkip: " << relinearizeSkip_ << std::endl;
        f << "enableRelinearization: " << (enableRelinearization_ ? "true" : "false") << std::endl;
        f << "evaluateNonLinearError: " << (evaluateNonlinearError_ ? "true" : "false") << std::endl;
        f << "factorization: ";
        switch (factorization_)
        {
        case gtsam::ISAM2Params::CHOLESKY:
            f << "CHOLESKY";
            break;
        case gtsam::ISAM2Params::QR:
            f << "QR";
            break;
        }
        f << std::endl;
        f << "cacheLinearizedFactors: " << (cacheLinearizedFactors_ ? "true" : "false") << std::endl;
    }
    else
    {
        LOG(ERROR) << "GtsamOptimizer could not open " << settingsFile;
    }
}
