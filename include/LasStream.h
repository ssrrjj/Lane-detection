#ifndef INCLUDE_LASSTREAM_H_
#define INCLUDE_INCLUDE_LASSTREAM_H__H_

#pragma once
#include "utils.h"
#include <fstream>
#include <memory>
#include <vector>
#include <string>
#include "las_consts_structs.h"

namespace las
{
    
    class BaseLasOutputStream
    {
    protected:
        std::ofstream file_stream_;

    public:
        BaseLasOutputStream(const std::string& file_name);
        virtual ~BaseLasOutputStream();

        bool isOpen() const;

        virtual void set_version(const float las_version) = 0;
        virtual void set_version(const uint8_t ver_major, const uint8_t ver_minor) = 0;
        virtual void set_system_identifier(const std::string& system_identifier) = 0;
        virtual void set_generating_software(const std::string& generating_software) = 0;
        virtual void set_file_creation_day(const uint16_t day) = 0;
        virtual void set_file_creation_year(const uint16_t year) = 0;
        virtual void set_point_format(const uint8_t point_format) = 0;
        virtual void set_scale_factor(const double scale_factor[3]) = 0;
        virtual void set_offset(const double offset[3]) = 0;

        virtual void writeHeader() = 0;
        virtual void writeVLR(const VLRHeader& header, const uint8_t record[], bool append = false) = 0;

        // write data into las file
        virtual void writeData(const double point[3], const uint16_t intensity, const double timestamp,
            const uint8_t num_returns, const uint8_t return_id) = 0;
        virtual void writeData(const double point[3], const uint16_t intensity, const double timestamp,
            const uint8_t num_returns, const uint8_t return_id, const uint16_t rgb[3]) = 0;
        virtual void writeData(const double point[3], const uint16_t intensity, const double timestamp,
            const uint8_t num_returns, const uint8_t return_id, const uint8_t classification) = 0;
        virtual void writeData(const double point[3], const uint16_t intensity, const double timestamp,
            const uint8_t num_returns, const uint8_t return_id, const double azimuth, const uint8_t flight_line_edge) = 0;
    };

    template <class T>
    class LasOutputStream : public BaseLasOutputStream
    {
    public:
        explicit LasOutputStream(const std::string& file_name);
        ~LasOutputStream();

        void set_version(const float las_version) override;
        void set_version(const uint8_t ver_major, const uint8_t ver_minor) override;
        void set_system_identifier(const std::string& system_identifier) override;
        void set_generating_software(const std::string& generating_software) override;
        void set_file_creation_day(const uint16_t day) override;
        void set_file_creation_year(const uint16_t year) override;
        void set_point_format(const uint8_t point_format) override;
        void set_scale_factor(const double scale_factor[3]) override;
        void set_offset(const double offset[3]) override;
        void set_header(const T& header) { header_ = header; }

        void writeHeader() override;
        void writeVLR(const VLRHeader& header, const uint8_t record[], bool append) override;

        void writeData(const double point[3], const uint16_t intensity, const double timestamp,
            const uint8_t num_returns, const uint8_t return_id) override;
        void writeData(const double point[3], const uint16_t intensity, const double timestamp,
            const uint8_t num_returns, const uint8_t return_id, const uint16_t rgb[3]) override;
        void writeData(const double point[3], const uint16_t intensity, const double timestamp,
            const uint8_t num_returns, const uint8_t return_id, const uint8_t classification) override;
        void writeData(const double point[3], const uint16_t intensity, const double timestamp,
            const uint8_t num_returns, const uint8_t return_id, const double azimuth, const uint8_t flight_line_edge) override;

        const T& get_header();

    protected:
        T header_;

        static uint16_t getPointDataSize(const uint8_t point_format);

        template <class P>
        void writePoint(const double point[3], const uint16_t intensity, const double timestamp,
            const uint8_t num_returns, const uint8_t return_id, P& las_point);
        template <class P>
        void writePoint(const double point[3], const uint16_t intensity, const double timestamp,
            const uint8_t num_returns, const uint8_t return_id, const uint16_t rgb[3], P& las_point);
        template <class P>
        void writePoint(const double point[3], const uint16_t intensity, const double timestamp,
            const uint8_t num_returns, const uint8_t return_id, const uint8_t classification, P& las_point);
        template <class P>
        void writePoint(const double point[3], const uint16_t intensity, const double timestamp,
            const uint8_t num_returns, const uint8_t return_id, const double azimuth, const uint8_t flight_line_edge, P& las_point);

        template <class P>
        friend class MultiLasOutputStream;
    };

    class BaseLasInputStream
    {
    protected:
        std::ifstream file_stream_;

    public:
        BaseLasInputStream(const std::string& file_name);
        virtual ~BaseLasInputStream();

        bool isOpen() const;

        virtual float get_version() = 0;
        virtual void get_version(uint8_t& ver_major, uint8_t& ver_minor) = 0;
        virtual std::string get_system_identifier() = 0;
        virtual std::string get_generating_software() = 0;
        virtual uint16_t get_file_creation_day() = 0;
        virtual uint16_t get_file_creation_year() = 0;
        virtual uint16_t get_header_size() = 0;
        virtual uint32_t get_offset_to_point_data() = 0;
        virtual uint32_t get_num_of_variable_length_records() = 0;
        virtual LasPointType get_point_format() = 0;
        virtual uint16_t get_point_data_length() = 0;
        virtual uint64_t get_num_of_points() = 0;
        virtual void get_num_of_points_by_returns(uint64_t nums[15]) = 0;
        virtual void get_scale_factor(double scale_factor[3]) = 0;
        virtual void get_offset(double offset[3]) = 0;
        virtual void get_point_range(double mins[3], double maxs[3]) = 0;
        virtual bool get_next_point(char point_data[]) = 0;
        virtual bool skip_next_point() = 0;
        virtual bool skip_points(uint64_t nom_points) = 0;
        // get VLR according to user_ID and record_ID or index
        virtual bool get_VLR(const char user_ID[16], const uint16_t record_ID, VLRHeader& VLR_header, std::vector<uint8_t>& record) = 0;
        virtual bool get_VLR(const uint32_t VLR_index, VLRHeader& VLR_header, std::vector<uint8_t>& record) = 0;

        virtual void resetStream() = 0;
    };


    template <class T>
    class LasInputStream : public BaseLasInputStream
    {
    public:
        explicit LasInputStream(const std::string& file_name);
        ~LasInputStream() = default;

        float get_version() override;
        void get_version(uint8_t& ver_major, uint8_t& ver_minor) override;
        std::string get_system_identifier() override;
        std::string get_generating_software() override;
        uint16_t get_file_creation_day() override;
        uint16_t get_file_creation_year() override;
        uint16_t get_header_size() override;
        uint32_t get_offset_to_point_data() override;
        uint32_t get_num_of_variable_length_records() override;
        LasPointType get_point_format() override;
        uint16_t get_point_data_length() override;
        uint64_t get_num_of_points() override;
        void get_num_of_points_by_returns(uint64_t nums[15]) override;
        void get_scale_factor(double scale_factor[3]) override;
        void get_offset(double offset[3]) override;
        void get_point_range(double mins[3], double maxs[3]) override;
        bool get_next_point(char point_data[]) override;
        bool skip_next_point() override;
        bool skip_points(uint64_t num_points) override;
        bool get_VLR(const char user_ID[16], const uint16_t record_ID, VLRHeader& VLR_header, std::vector<uint8_t>& record) override;
        bool get_VLR(const uint32_t VLR_index, VLRHeader& VLR_header, std::vector<uint8_t>& record) override;

        void resetStream();

        template <class P>
        bool get_next_point(P& las_point);

        bool isHeaderValid(std::string& error_msg);
        const T& get_header();

    protected:
        T header_;
        uint8_t header_valid_;
        uint64_t point_index_;

        static uint8_t checkHeaderFields(const T& header);
        static bool checkPointFormatAndSize(const uint8_t point_type, const uint16_t point_size);

        template <class P>
        static bool matchPointFormat(const P& las_point, const LasPointType& point_type);
    };

    class BaseMultiLasOutputStream
    {
    public:
        explicit BaseMultiLasOutputStream() = default;
        virtual ~BaseMultiLasOutputStream() = default;

        virtual bool isOpen() = 0;

        virtual void writeVLR(const VLRHeader& header, const uint8_t record[], bool append = false) = 0;

        // write data into las file
        virtual bool writeData(const double point[3], const uint8_t intensity, const double timestamp,
            const uint8_t num_returns, const uint8_t return_id) = 0;
        virtual bool writeData(const double point[3], const uint8_t intensity, const double timestamp,
            const uint8_t num_returns, const uint8_t return_id, const uint16_t rgb[3]) = 0;
        virtual bool writeData(const double point[3], const uint8_t intensity, const double timestamp,
            const uint8_t num_returns, const uint8_t return_id, const uint8_t rms) = 0;
        virtual bool writeData(const double point[3], const uint8_t intensity, const double timestamp,
            const uint8_t num_returns, const uint8_t return_id, const double azimuth, const uint8_t flight_line_edge) = 0;

        virtual std::shared_ptr<BaseLasOutputStream> get_las_stream_ptr() = 0;
    };

    template <class T>
    class MultiLasOutputStream : public BaseMultiLasOutputStream
    {
    public:
        explicit MultiLasOutputStream(const std::string& file_name, bool split_enabled, int max_file_size,
            uint16_t max_split_allowed = 10000, uint8_t index_padding = 5);
        ~MultiLasOutputStream() = default;

        bool isOpen();

        void writeVLR(const VLRHeader& header, const uint8_t record[], bool append = false);

        // write data into las file
        bool writeData(const double point[3], const uint8_t intensity, const double timestamp,
            const uint8_t num_returns, const uint8_t return_id);
        bool writeData(const double point[3], const uint8_t intensity, const double timestamp,
            const uint8_t num_returns, const uint8_t return_id, const uint16_t rgb[3]);
        bool writeData(const double point[3], const uint8_t intensity, const double timestamp,
            const uint8_t num_returns, const uint8_t return_id, const uint8_t rms);
        bool writeData(const double point[3], const uint8_t intensity, const double timestamp,
            const uint8_t num_returns, const uint8_t return_id, const double azimuth, const uint8_t flight_line_edge);

        std::shared_ptr<BaseLasOutputStream> get_las_stream_ptr()
        {
            return std::dynamic_pointer_cast<BaseLasOutputStream>(las_stream_ptr_);
        }

    protected:
        std::shared_ptr<LasOutputStream<T>> las_stream_ptr_;

        bool split_enabled_;

        std::string file_name_;
        int file_index_;
        int max_file_size_;  // in mb
        uint64_t max_num_points_;

        uint16_t max_split_allowed_;
        uint8_t index_padding_;

        // VLR buffer
        std::vector<VLRHeader> VLRHeaderBuffer;
        std::vector<uint8_t> VLRRecordBuffer;

        void calculateMaxNumPoints();
        void newLasStream();
    };
    
}
void readlas(std::string file, pcl::PointCloud<pcl::PointXYZI>::Ptr ret);

#endif /* INCLUDE_UTILS_H_ */
