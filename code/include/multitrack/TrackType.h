#pragma once

#include "cinder/Surface.h"
#include "cinder/ImageIo.h"
#include "cinder/Utilities.h"

#include "Kinect2.h"

#include <multitrack/Timer.h>

namespace itp { namespace multitrack {

	typedef std::shared_ptr<struct PointCloud> PointCloudRef;

	struct PointCloud
	{
		std::deque<ci::vec2> mPoints;

		PointCloud()
		{
			/* no-op */
		}

		PointCloud(const Kinect2::BodyFrame& frame, const Kinect2::DeviceRef& device, bool includeAll = true)
		{
			for (const Kinect2::Body& body : frame.getBodies()) {
				if (body.isTracked()) {
					for (const auto& joint : body.getJointMap()) {
						if (includeAll || joint.second.getTrackingState() == TrackingState::TrackingState_Tracked) {
							mPoints.push_back(device->mapCameraToDepth(joint.second.getPosition()));
						}
					}
				}
			}
		}
	};

	template<typename T> inline std::string get_file_extension() { /* no-op */ }
	template<typename T> inline T read_from_file(const ci::fs::path& inputPath) { /* no-op */ }
	template<typename T> inline void write_to_file(const ci::fs::path& outputPath, const T& outputItem) { /* no-op */ }

	template<> inline std::string get_file_extension<ci::SurfaceRef>()
	{
		return "png";
	}

	template<> inline ci::SurfaceRef read_from_file<ci::SurfaceRef>(const ci::fs::path& inputPath)
	{
		return ci::Surface::create(ci::loadImage(inputPath));
	}

	template<> inline void write_to_file<ci::SurfaceRef>(const ci::fs::path& outputPath, const ci::SurfaceRef& outputItem)
	{
		ci::writeImage(outputPath, *outputItem);
	}

	template<> inline std::string get_file_extension<PointCloudRef>()
	{
		return "txt";
	}

	template<> inline PointCloudRef read_from_file<PointCloudRef>(const ci::fs::path& inputPath)
	{
		// Try to open file:
		std::ifstream tFile(inputPath.string());
		// Handle info file:
		if (tFile.is_open()) {
			// Initialize output:
			PointCloudRef tOutput = std::make_shared<PointCloud>();
			// Iterate over each line in info file:
			std::string tTemp;
			while (std::getline(tFile, tTemp)) {
				// Find delimiter:
				std::size_t tFind = tTemp.find_first_of(' ');
				// Handle frame:
				if (tFind != std::string::npos) {
					tOutput->mPoints.push_back(ci::vec2(atof(tTemp.substr(0, tFind).c_str()), atof(tTemp.substr(tFind + 1).c_str())));
				}
				// Handle error:
				else {
					throw std::runtime_error("Could not read file: \'" + inputPath.string() + "\'");
				}
			}
			// Close info file:
			tFile.close();
			// Return point cloud:
			return tOutput;
		}
		// Handle file-open error:
		throw std::runtime_error("Could not open file: \'" + inputPath.string() + "\'");
	}

	template<> inline void write_to_file<PointCloudRef>(const ci::fs::path& outputPath, const PointCloudRef& outputItem)
	{
		std::ofstream tFile;
		tFile.open(outputPath.string());
		for (const auto& pt : outputItem->mPoints) {
			tFile << pt.x << ' ' << pt.y << std::endl;
		}
		tFile.close();
	}

} } // namespace itp::multitrack
