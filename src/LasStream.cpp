#include <string>
#include <algorithm>
#include <sstream>
#include <iomanip>

#include "LasStream.h"
#include <pcl/point_types.h>
#include <pcl/common/distances.h>
#include "data_structure.h"
#include "las_consts_structs.h"
#include "PointXYZIT.h"
using namespace std;

namespace las
{
    BaseLasOutputStream::BaseLasOutputStream(const string& file_name)
    {
        // open file stream
        file_stream_.open(file_name, ios::binary);
    }

    BaseLasOutputStream::~BaseLasOutputStream()
    {
        // close file stream
        if (file_stream_.is_open())
        {
            file_stream_.close();
        }
    }

    bool BaseLasOutputStream::isOpen() const
    {
        return file_stream_.is_open();
    }

    // ============================================================================================================================================

    template <class T>
    LasOutputStream<T>::LasOutputStream(const string& file_name) : BaseLasOutputStream(file_name)
    {
        memset(&header_, 0, sizeof(header_));
        header_.file_signature[0] = 'L';
        header_.file_signature[1] = 'A';
        header_.file_signature[2] = 'S';
        header_.file_signature[3] = 'F';
        header_.global_encoding = 0;
        header_.header_size = sizeof(header_);
        header_.offset_to_point_data = sizeof(header_);
    }

    template <class T>
    LasOutputStream<T>::~LasOutputStream()
    {
        if (file_stream_ && file_stream_.is_open())
            writeHeader();
    }

    template <class T>
    void LasOutputStream<T>::set_version(const float las_version)
    {
        header_.version_major = static_cast<uint8_t>(las_version);
        header_.version_minor = static_cast<uint8_t>(round((las_version - header_.version_major) * 10));
    }

    template <class T>
    void LasOutputStream<T>::set_version(const uint8_t ver_major, const uint8_t ver_minor)
    {
        header_.version_major = ver_major;
        header_.version_minor = ver_minor;
    }

    template <class T>
    void LasOutputStream<T>::set_system_identifier(const std::string& system_identifier)
    {
        strcpy(header_.system_identifier, system_identifier.c_str());
    }

    template <class T>
    void LasOutputStream<T>::set_generating_software(const std::string& generating_software)
    {
        strcpy(header_.generating_software, generating_software.c_str());
    }
    
    template <class T>
    void LasOutputStream<T>::set_file_creation_day(const uint16_t day)
    {
        header_.file_creation_day = day;
    }

    template <class T>
    void LasOutputStream<T>::set_file_creation_year(const uint16_t year)
    {
        header_.file_creation_year = year;
    }

    template <class T>
    uint16_t LasOutputStream<T>::getPointDataSize(const uint8_t point_format)
    {
        switch (LasPointType(point_format))
        {
        case LasPointType::LasPoint0: return sizeof(LasPoint0);
        case LasPointType::LasPoint1: return sizeof(LasPoint1);
        case LasPointType::LasPoint3: return sizeof(LasPoint3);
        case LasPointType::LasPoint6: return sizeof(LasPoint6);
        case LasPointType::LasPoint7: return sizeof(LasPoint7);
        default: return 0;
        }
    }

    template <class T>
    void LasOutputStream<T>::set_point_format(const uint8_t point_format)
    {
        header_.point_data_record_format = point_format;
        header_.point_data_record_length = getPointDataSize(point_format);
    }

    template <class T>
    void LasOutputStream<T>::set_scale_factor(const double scale_factor[3])
    {
        memcpy(&header_.x_scale_factor, scale_factor, 3 * sizeof(double));
    }

    template <class T>
    void LasOutputStream<T>::set_offset(const double offset[3])
    {
        memcpy(&header_.x_offset, offset, 3 * sizeof(double));
    }

    template <class T>
    void LasOutputStream<T>::writeHeader()
    {
        file_stream_.seekp(0, std::ios::beg);
        file_stream_.write(reinterpret_cast<char*>(&header_), sizeof(header_));
    }

    template <class T>
    void LasOutputStream<T>::writeVLR(const VLRHeader& VLR_header, const uint8_t record[], bool append)
    {
        // if append is false, the file stream will be relocated to the end of the last VLR
        // otherwise, it will assume the file stream is at the end of the last VLR 
        // VLR writing should be done before writing any point data
        if (!append)
            file_stream_.seekp(header_.offset_to_point_data, file_stream_.beg);

        file_stream_.write(reinterpret_cast<const char*>(&VLR_header), sizeof(VLR_header));
        file_stream_.write(reinterpret_cast<const char*>(record), VLR_header.record_length);
        header_.offset_to_point_data += sizeof(VLR_header) + VLR_header.record_length;
        ++header_.number_of_variable_length_records;
    }

    template <class T>
    void LasOutputStream<T>::writeData(const double point[3], const uint16_t intensity, const double timestamp,
        const uint8_t num_returns, const uint8_t return_id)
    {
        if (header_.number_of_point_records == 0)
        {
            header_.min_x = header_.max_x = point[0];
            header_.min_y = header_.max_y = point[1];
            header_.min_z = header_.max_z = point[2];
            memcpy(&header_.x_offset, point, 3 * sizeof(double));
        }
        else
        {
            header_.min_x = std::min(header_.min_x, point[0]);
            header_.max_x = std::max(header_.max_x, point[0]);
            header_.min_y = std::min(header_.min_y, point[1]);
            header_.max_y = std::max(header_.max_y, point[1]);
            header_.min_z = std::min(header_.min_z, point[2]);
            header_.max_z = std::max(header_.max_z, point[2]);
        }

        switch (LasPointType(header_.point_data_record_format))
        {
        case LasPointType::LasPoint1:
        {
            LasPoint1 las_point;
            writePoint(point, intensity, timestamp, num_returns, return_id, las_point);
            file_stream_.write(reinterpret_cast<char*>(&las_point), sizeof(las_point));
            break;
        }
        case LasPointType::LasPoint3:
        {
            LasPoint3 las_point;
            writePoint(point, intensity, timestamp, num_returns, return_id, las_point);
            file_stream_.write(reinterpret_cast<char*>(&las_point), sizeof(las_point));
            break;
        }
        case LasPointType::LasPoint6:
        {
            LasPoint6 las_point;
            writePoint(point, intensity, timestamp, num_returns, return_id, las_point);
            file_stream_.write(reinterpret_cast<char*>(&las_point), sizeof(las_point));
            break;
        }
        case LasPointType::LasPoint7:
        {
            LasPoint7 las_point;
            writePoint(point, intensity, timestamp, num_returns, return_id, las_point);
            file_stream_.write(reinterpret_cast<char*>(&las_point), sizeof(las_point));
            break;
        }
        default:
            break;
        }
    }

    template <class T>
    void LasOutputStream<T>::writeData(const double point[3], const uint16_t intensity, const double timestamp,
        const uint8_t num_returns, const uint8_t return_id, const uint16_t rgb[3])
    {
        if (header_.number_of_point_records == 0)
        {
            header_.min_x = header_.max_x = point[0];
            header_.min_y = header_.max_y = point[1];
            header_.min_z = header_.max_z = point[2];
            memcpy(&header_.x_offset, point, 3 * sizeof(double));
        }
        else
        {
            header_.min_x = std::min(header_.min_x, point[0]);
            header_.max_x = std::max(header_.max_x, point[0]);
            header_.min_y = std::min(header_.min_y, point[1]);
            header_.max_y = std::max(header_.max_y, point[1]);
            header_.min_z = std::min(header_.min_z, point[2]);
            header_.max_z = std::max(header_.max_z, point[2]);
        }

        switch (LasPointType(header_.point_data_record_format))
        {
        case LasPointType::LasPoint1:
        {
            LasPoint1 las_point;
            writePoint(point, intensity, timestamp, num_returns, return_id, las_point);
            file_stream_.write(reinterpret_cast<char*>(&las_point), sizeof(las_point));
            break;
        }
        case LasPointType::LasPoint3:
        {
            LasPoint3 las_point;
            writePoint(point, intensity, timestamp, num_returns, return_id, rgb, las_point);
            file_stream_.write(reinterpret_cast<char*>(&las_point), sizeof(las_point));
            break;
        }
        case LasPointType::LasPoint6:
        {
            LasPoint6 las_point;
            writePoint(point, intensity, timestamp, num_returns, return_id, las_point);
            file_stream_.write(reinterpret_cast<char*>(&las_point), sizeof(las_point));
            break;
        }
        case LasPointType::LasPoint7:
        {
            LasPoint7 las_point;
            writePoint(point, intensity, timestamp, num_returns, return_id, rgb, las_point);
            file_stream_.write(reinterpret_cast<char*>(&las_point), sizeof(las_point));
            break;
        }
        default:
            break;
        }
    }

    template <class T>
    void LasOutputStream<T>::writeData(const double point[3], const uint16_t intensity, const double timestamp,
        const uint8_t num_returns, const uint8_t return_id, const uint8_t classification)
    {
        if (header_.number_of_point_records == 0)
        {
            header_.min_x = header_.max_x = point[0];
            header_.min_y = header_.max_y = point[1];
            header_.min_z = header_.max_z = point[2];
            memcpy(&header_.x_offset, point, 3 * sizeof(double));
        }
        else
        {
            header_.min_x = std::min(header_.min_x, point[0]);
            header_.max_x = std::max(header_.max_x, point[0]);
            header_.min_y = std::min(header_.min_y, point[1]);
            header_.max_y = std::max(header_.max_y, point[1]);
            header_.min_z = std::min(header_.min_z, point[2]);
            header_.max_z = std::max(header_.max_z, point[2]);
        }

        switch (LasPointType(header_.point_data_record_format))
        {
        case LasPointType::LasPoint1:
        {
            LasPoint1 las_point;
            writePoint(point, intensity, timestamp, num_returns, return_id, classification, las_point);
            file_stream_.write(reinterpret_cast<char*>(&las_point), sizeof(las_point));
            break;
        }
        case LasPointType::LasPoint3:
        {
            LasPoint3 las_point;
            writePoint(point, intensity, timestamp, num_returns, return_id, classification, las_point);
            file_stream_.write(reinterpret_cast<char*>(&las_point), sizeof(las_point));
            break;
        }
        case LasPointType::LasPoint6:
        {
            LasPoint6 las_point;
            writePoint(point, intensity, timestamp, num_returns, return_id, classification, las_point);
            file_stream_.write(reinterpret_cast<char*>(&las_point), sizeof(las_point));
            break;
        }
        case LasPointType::LasPoint7:
        {
            LasPoint7 las_point;
            writePoint(point, intensity, timestamp, num_returns, return_id, classification, las_point);
            file_stream_.write(reinterpret_cast<char*>(&las_point), sizeof(las_point));
            break;
        }
        default:
            break;
        }
    }

    template <class T>
    void LasOutputStream<T>::writeData(const double point[3], const uint16_t intensity, const double timestamp,
        const uint8_t num_returns, const uint8_t return_id, const double azimuth, const uint8_t flight_line_edge)
    {
        if (header_.number_of_point_records == 0)
        {
            header_.min_x = header_.max_x = point[0];
            header_.min_y = header_.max_y = point[1];
            header_.min_z = header_.max_z = point[2];
            memcpy(&header_.x_offset, point, 3 * sizeof(double));
        }
        else
        {
            header_.min_x = std::min(header_.min_x, point[0]);
            header_.max_x = std::max(header_.max_x, point[0]);
            header_.min_y = std::min(header_.min_y, point[1]);
            header_.max_y = std::max(header_.max_y, point[1]);
            header_.min_z = std::min(header_.min_z, point[2]);
            header_.max_z = std::max(header_.max_z, point[2]);
        }

        switch (LasPointType(header_.point_data_record_format))
        {
        case LasPointType::LasPoint1:
        {
            LasPoint1 las_point;
            writePoint(point, intensity, timestamp, num_returns, return_id, 0, flight_line_edge, las_point);
            file_stream_.write(reinterpret_cast<char*>(&las_point), sizeof(las_point));
            break;
        }
        case LasPointType::LasPoint3:
        {
            LasPoint3 las_point;
            writePoint(point, intensity, timestamp, num_returns, return_id, 0, flight_line_edge, las_point);
            file_stream_.write(reinterpret_cast<char*>(&las_point), sizeof(las_point));
            break;
        }
        case LasPointType::LasPoint6:
        {
            LasPoint6 las_point;
            writePoint(point, intensity, timestamp, num_returns, return_id, azimuth, flight_line_edge, las_point);
            file_stream_.write(reinterpret_cast<char*>(&las_point), sizeof(las_point));
            break;
        }
        case LasPointType::LasPoint7:
        {
            LasPoint7 las_point;
            writePoint(point, intensity, timestamp, num_returns, return_id, azimuth, flight_line_edge, las_point);
            file_stream_.write(reinterpret_cast<char*>(&las_point), sizeof(las_point));
            break;
        }
        default:
            break;
        }
    }

    template <class T>
    template <class P>
    void LasOutputStream<T>::writePoint(const double point[3], const uint16_t intensity, const double timestamp,
        const uint8_t num_returns, const uint8_t return_id, P& las_point)
    {
        memset(&las_point, 0, sizeof(las_point));

        // set xyz
        int scale_point[3];
        scale_point[0] = static_cast<int>((point[0] - header_.x_offset) / header_.x_scale_factor);
        scale_point[1] = static_cast<int>((point[1] - header_.y_offset) / header_.y_scale_factor);
        scale_point[2] = static_cast<int>((point[2] - header_.z_offset) / header_.z_scale_factor);
        memcpy(&las_point.x, scale_point, sizeof(scale_point));

        // set the rest
        //las_point.intensity = intensity * LAS_INTENSITY_SCALE;
        las_point.intensity = intensity;
        las_point.GPS_time = timestamp;
        las_point.num_returns = num_returns;
        las_point.return_num = return_id;

        header_.number_of_point_records++;
        header_.number_of_points_by_return[return_id - 1]++;
    }

    template <class T>
    template <class P>
    void LasOutputStream<T>::writePoint(const double point[3], const uint16_t intensity, const double timestamp,
        const uint8_t num_returns, const uint8_t return_id, const uint16_t rgb[3], P& las_point)
    {
        writePoint(point, intensity, timestamp, num_returns, return_id, las_point);

        // set rgb
        if (rgb != nullptr)
        {
            memcpy(&las_point.red, rgb, 3 * sizeof(uint16_t));
        }
    }

    template <class T>
    template <class P>
    void LasOutputStream<T>::writePoint(const double point[3], const uint16_t intensity, const double timestamp,
        const uint8_t num_returns, const uint8_t return_id, const uint8_t classification, P& las_point)
    {
        writePoint(point, intensity, timestamp, num_returns, return_id, las_point);
        las_point.classification = classification;
    }

    template <class T>
    template <class P>
    void LasOutputStream<T>::writePoint(const double point[3], const uint16_t intensity, const double timestamp,
        const uint8_t num_returns, const uint8_t return_id, const double azimuth, const uint8_t flight_line_edge, P& las_point)
    {
        writePoint(point, intensity, timestamp, num_returns, return_id, las_point);
        las_point.scan_angle = static_cast<int16_t>((azimuth > 180 ? azimuth - 360 : azimuth) / LAS_SCAN_ANGLE_1_4);
        las_point.edge_of_flight_line = flight_line_edge;
    }

    template <class T>
    const T& LasOutputStream<T>::get_header()
    {
        return header_;
    }

    template class LasOutputStream<LegacyLasHeader>;
    template class LasOutputStream<LasHeader>;

    // ============================================================================================================================================

    BaseLasInputStream::BaseLasInputStream(const string& file_name)
    {
        // open file stream
        file_stream_.open(file_name, ios::binary);
    }

    BaseLasInputStream::~BaseLasInputStream()
    {
        // close file stream
        if (file_stream_.is_open())
        {
            file_stream_.close();
        }
    }

    bool BaseLasInputStream::isOpen() const
    {
        return file_stream_.is_open();
    }

    // ============================================================================================================================================

    template <class T>
    LasInputStream<T>::LasInputStream(const std::string& file_name) : BaseLasInputStream(file_name), point_index_(0)
    {
        file_stream_.read(reinterpret_cast<char*>(&header_), sizeof(header_));
        header_valid_ = file_stream_.gcount() != sizeof(header_) ? 6 : 0;
        if (header_valid_ == 0)
        {
            header_valid_ = checkHeaderFields(header_);
        }
    }

    template <class T>
    bool LasInputStream<T>::checkPointFormatAndSize(const uint8_t point_type, const uint16_t point_size)
    {
        switch (point_type)
        {
        case as_int(LasPointType::LasPoint0):
            return point_size == sizeof(LasPoint0);
        case as_int(LasPointType::LasPoint1):
            return point_size == sizeof(LasPoint1);
        case as_int(LasPointType::LasPoint3):
            return point_size == sizeof(LasPoint3);
        case as_int(LasPointType::LasPoint6):
            return point_size == sizeof(LasPoint6);
        case as_int(LasPointType::LasPoint7):
            return point_size == sizeof(LasPoint7);
        default:
            return false;
        }
    }

    template <class T>
    uint8_t LasInputStream<T>::checkHeaderFields(const T& header)
    {
        // check las version
        if (header.version_major > LATEST_VER_MAJOR
            || header.version_major == LATEST_VER_MAJOR && header.version_minor > LATEST_VER_MINOR)
            return 1;
        // check point coordinate range
        if (header.min_x > header.max_x || header.min_y > header.max_y || header.min_z > header.max_z)
            return 2;
        // check header size
        if (header.header_size != sizeof(T))
            return 3;
        // check offset to point data
        if (header.offset_to_point_data < header.header_size)
            return 4;
        // check point format and size
        if (!checkPointFormatAndSize(header.point_data_record_format, header.point_data_record_length))
            return 5;
        return 0;
    }

    template <class T>
    bool LasInputStream<T>::isHeaderValid(std::string& error_msg)
    {
        if (header_valid_ == 0)
            return true;

        switch (header_valid_)
        {
        case 1: error_msg = "LAS: version is too new. Accept version no newer than "
            + to_string(LATEST_VER_MAJOR) + "." + to_string(LATEST_VER_MINOR); break;
        case 2: error_msg = "LAS: point coordinate range is invalid"; break;
        case 3: error_msg = "LAS: header size is invalid"; break;
        case 4: error_msg = "LAS: offset to point data is invalid"; break;
        case 5: error_msg = "LAS: point format and size are invalid"; break;
        case 6: error_msg = "LAS: file incomplete. Couldn't read full header"; break;
        default: break;
        }
        return false;
    }

    template <class T>
    const T& LasInputStream<T>::get_header()
    {
        return header_;
    }

    template <class T>
    float LasInputStream<T>::get_version()
    {
        float ver = header_.version_major;
        float minor = header_.version_minor;
        while (minor >= 1)
            minor /= 10;
        ver += minor;
        return ver;
    }

    template <class T>
    void LasInputStream<T>::get_version(uint8_t& ver_major, uint8_t& ver_minor)
    {
        ver_major = header_.version_major;
        ver_minor = header_.version_minor;
    }

    template <class T>
    std::string LasInputStream<T>::get_system_identifier()
    {
        return header_.system_identifier;
    }

    template <class T>
    std::string LasInputStream<T>::get_generating_software()
    {
        return header_.generating_software;
    }

    template <class T>
    uint16_t LasInputStream<T>::get_file_creation_day()
    {
        return header_.file_creation_day;
    }

    template <class T>
    uint16_t LasInputStream<T>::get_file_creation_year()
    {
        return header_.file_creation_year;
    }

    template <class T>
    LasPointType LasInputStream<T>::get_point_format()
    {
        return LasPointType(header_.point_data_record_format);
    }

    template <class T>
    uint16_t LasInputStream<T>::get_point_data_length()
    {
        return header_.point_data_record_length;
    }

    template <class T>
    uint64_t LasInputStream<T>::get_num_of_points()
    {
        return header_.number_of_point_records;
    }

    template <class T>
    uint16_t LasInputStream<T>::get_header_size()
    {
        return header_.header_size;
    }

    template <class T>
    uint32_t LasInputStream<T>::get_offset_to_point_data()
    {
        return header_.offset_to_point_data;
    }

    template <class T>
    uint32_t LasInputStream<T>::get_num_of_variable_length_records()
    {
        return header_.number_of_variable_length_records;
    }

    template <class T>
    void LasInputStream<T>::get_num_of_points_by_returns(uint64_t nums[15])
    {
        memset(nums, 0, sizeof(uint64_t) * 15);
        if (std::is_same<T, LegacyLasHeader>::value)
        {
            for (int i = 0; i < 5; ++i)
                nums[i] = header_.number_of_points_by_return[i];
        }
        else if (std::is_same<T, LasHeader>::value)
        {
            memcpy(nums, header_.number_of_points_by_return, sizeof(uint64_t) * 15);
        }
        else
        {
#ifdef INTERNAL_DEBUGGING
            printf("Invalid LAS header type.\n");
#endif
        }
    }

    template <class T>
    void LasInputStream<T>::get_scale_factor(double scale_factor[3])
    {
        memcpy(scale_factor, &header_.x_scale_factor, 3 * sizeof(double));
    }

    template <class T>
    void LasInputStream<T>::get_offset(double offset[3])
    {
        memcpy(offset, &header_.x_offset, 3 * sizeof(double));
    }

    template <class T>
    void LasInputStream<T>::get_point_range(double mins[3], double maxs[3])
    {
        mins[0] = header_.min_x;
        mins[1] = header_.min_y;
        mins[2] = header_.min_z;

        maxs[0] = header_.max_x;
        maxs[1] = header_.max_y;
        maxs[2] = header_.max_z;
    }

    template <class T>
    template <class P>
    bool LasInputStream<T>::matchPointFormat(const P& las_point, const LasPointType& point_type)
    {
        switch (point_type)
        {
        case LasPointType::LasPoint0:
            return std::is_same<P, LasPoint0>::value;
        case LasPointType::LasPoint1:
            return std::is_same<P, LasPoint1>::value;
        case LasPointType::LasPoint3:
            return std::is_same<P, LasPoint3>::value;
        case LasPointType::LasPoint6:
            return std::is_same<P, LasPoint6>::value;
        case LasPointType::LasPoint7:
            return std::is_same<P, LasPoint7>::value;
        default:
            return false;
        }
    }

    template <class T>
    template <class P>
    bool LasInputStream<T>::get_next_point(P& las_point)
    {
        if (!matchPointFormat(las_point, LasPointType(header_.point_data_record_format)))
        {
#ifdef INTERNAL_DEBUGGING
            printf("Point format doesn't match with Las point.\n");
#endif
            return false;
        }

        if (point_index_ >= header_.number_of_point_records)
        {
#ifdef INTERNAL_DEBUGGING
            printf("No more LAS points\n");
#endif
            return false;
        }

        if (point_index_ == 0)
        {
            file_stream_.seekg(header_.offset_to_point_data, file_stream_.beg);
        }

        file_stream_.read(reinterpret_cast<char*>(&las_point), header_.point_data_record_length);

        if (!file_stream_.good() || file_stream_.gcount() != header_.point_data_record_length)
        {
#ifdef INTERNAL_DEBUGGING
            printf("File stream error. Can't read the next point.\n");
#endif
            return false;
        }

        ++point_index_;
        return true;
    }

    template <class T>
    bool LasInputStream<T>::get_next_point(char point_data[])
    {
        if (point_index_ >= header_.number_of_point_records)
        {
#ifdef INTERNAL_DEBUGGING
            printf("No more LAS points\n");
#endif
            return false;
        }

        if (point_index_ == 0)
        {
            file_stream_.seekg(header_.offset_to_point_data, file_stream_.beg);
        }

        file_stream_.read(point_data, header_.point_data_record_length);

        if (!file_stream_.good() || file_stream_.gcount() != header_.point_data_record_length)
        {
#ifdef INTERNAL_DEBUGGING
            printf("File stream error. Can't read the next point.\n");
#endif
            return false;
        }

        ++point_index_;
        return true;
    }

    template <class T>
    bool LasInputStream<T>::skip_next_point()
    {
        if (point_index_ >= header_.number_of_point_records)
        {
#ifdef INTERNAL_DEBUGGING
            printf("No more LAS points\n");
#endif
            return false;
        }

        if (point_index_ == 0)
        {
            file_stream_.seekg(header_.offset_to_point_data, file_stream_.beg);
        }

        file_stream_.seekg(header_.point_data_record_length, file_stream_.cur);

        if (!file_stream_.good())
        {
#ifdef INTERNAL_DEBUGGING
            printf("File stream error. Can't skip the next point.\n");
#endif
            return false;
        }

        ++point_index_;
        return true;
    }

    template <class T>
    bool LasInputStream<T>::skip_points(uint64_t num_points)
    {
        if (point_index_ >= header_.number_of_point_records)
        {
#ifdef INTERNAL_DEBUGGING
            printf("No more LAS points\n");
#endif
            return false;
        }

        if (point_index_ == 0)
        {
            file_stream_.seekg(header_.offset_to_point_data, file_stream_.beg);
        }

        uint64_t to_skip = min(num_points, header_.number_of_point_records - point_index_);
        file_stream_.seekg(to_skip * header_.point_data_record_length, file_stream_.cur);

        if (!file_stream_ || file_stream_.eof())
        {
#ifdef INTERNAL_DEBUGGING
            printf("File stream error. Can't skip %llu point.\n", num_points);
#endif
            return false;
        }

        point_index_ += to_skip;
        return true;
    }

    template <class T>
    bool LasInputStream<T>::get_VLR(const char user_ID[16], const uint16_t record_ID, VLRHeader& VLR_header, vector<uint8_t>& record)
    {
        auto pos = file_stream_.tellg();
        file_stream_.seekg(header_.header_size, file_stream_.beg);
        bool found = false;
        for (uint32_t i = 0; i < header_.number_of_variable_length_records; ++i)
        {
            file_stream_.read(reinterpret_cast<char*>(&VLR_header), sizeof(VLR_header));
            if (strcmp(VLR_header.user_ID, user_ID) == 0 && record_ID == VLR_header.record_ID)
            {
                record.reserve(VLR_header.record_length);
                file_stream_.read(reinterpret_cast<char*>(record.data()), VLR_header.record_length);
                found = true;
                break;
            }
            else
            {
                file_stream_.seekg(VLR_header.record_length);
            }
        }
        file_stream_.seekg(pos);
        return found;
    }

    template <class T>
    bool LasInputStream<T>::get_VLR(const uint32_t VLR_index, VLRHeader& VLR_header, vector<uint8_t>& record)
    {
        if (VLR_index >= header_.number_of_variable_length_records) { return false; }

        auto pos = file_stream_.tellg();
        file_stream_.seekg(header_.header_size, file_stream_.beg);
        for (uint32_t i = 0; i < VLR_index; ++i)
        {
            file_stream_.read(reinterpret_cast<char*>(&VLR_header), sizeof(VLR_header));
            file_stream_.ignore(VLR_header.record_length);
        }
        file_stream_.read(reinterpret_cast<char*>(&VLR_header), sizeof(VLR_header));
        record.reserve(VLR_header.record_length);
        file_stream_.read(reinterpret_cast<char*>(record.data()), VLR_header.record_length);
        file_stream_.seekg(pos);
        return true;
    }

    template <class T>
    void LasInputStream<T>::resetStream()
    {
        point_index_ = 0;
    }

    template class LasInputStream<LegacyLasHeader>;
    template class LasInputStream<LasHeader>;

    template bool LasInputStream<LegacyLasHeader>::get_next_point(LasPoint1&);
    template bool LasInputStream<LegacyLasHeader>::get_next_point(LasPoint3&);
    template bool LasInputStream<LegacyLasHeader>::get_next_point(LasPoint6&);
    template bool LasInputStream<LegacyLasHeader>::get_next_point(LasPoint7&);

    template bool LasInputStream<LasHeader>::get_next_point(LasPoint1&);
    template bool LasInputStream<LasHeader>::get_next_point(LasPoint3&);
    template bool LasInputStream<LasHeader>::get_next_point(LasPoint6&);
    template bool LasInputStream<LasHeader>::get_next_point(LasPoint7&);

    // ============================================================================================================================================

    template <class T>
    MultiLasOutputStream<T>::MultiLasOutputStream(const std::string& file_name, bool split_enabled, int max_file_size,
        uint16_t max_split_allowed, uint8_t index_padding)
    {
        split_enabled_ = split_enabled;

        file_name_ = file_name;
        file_index_ = 0;
        max_file_size_ = max_file_size;
        max_num_points_ = 0;
        max_split_allowed_ = max_split_allowed;
        index_padding_ = index_padding;

        if (split_enabled_)
        {
            ostringstream name_stream;
            name_stream << file_name_.substr(0, file_name_.find_last_of(".")) << "_"
                << setfill('0') << setw(index_padding_) << file_index_ << ".las";
            las_stream_ptr_ = make_shared<LasOutputStream<T>>(name_stream.str());
        }
        else
            las_stream_ptr_ = make_shared<LasOutputStream<T>>(file_name_);
    }

    template <class T>
    bool MultiLasOutputStream<T>::isOpen()
    {
        return las_stream_ptr_ && las_stream_ptr_->isOpen();
    }

    template <class T>
    void MultiLasOutputStream<T>::writeVLR(const VLRHeader& header, const uint8_t record[], bool append)
    {
        VLRHeaderBuffer.push_back(header);
        std::copy(record, record + header.record_length, back_inserter(VLRRecordBuffer));

        if (las_stream_ptr_)
            las_stream_ptr_->writeVLR(header, record, append);
    }

    template <class T>
    bool MultiLasOutputStream<T>::writeData(const double point[3], const uint8_t intensity, const double timestamp,
        const uint8_t num_returns, const uint8_t return_id)
    {
        if (!las_stream_ptr_) return false;
        auto& header = las_stream_ptr_->header_;

        if (max_num_points_ == 0)
            calculateMaxNumPoints();

        if (split_enabled_ && header.number_of_point_records >= max_num_points_ && max_num_points_ > 0)
        {
            if (file_index_ < max_split_allowed_) newLasStream();
            if (!las_stream_ptr_->isOpen()) return false;
        }

        las_stream_ptr_->writeData(point, intensity, timestamp, num_returns, return_id);

        return true;
    }

    template <class T>
    bool MultiLasOutputStream<T>::writeData(const double point[3], const uint8_t intensity, const double timestamp,
        const uint8_t num_returns, const uint8_t return_id, const uint16_t rgb[3])
    {
        if (!las_stream_ptr_) return false;
        auto& header = las_stream_ptr_->header_;

        if (max_num_points_ == 0)
            calculateMaxNumPoints();

        if (split_enabled_ && header.number_of_point_records >= max_num_points_ && max_num_points_ > 0)
        {
            if (file_index_ < max_split_allowed_) newLasStream();
            if (!las_stream_ptr_->isOpen()) return false;
        }

        las_stream_ptr_->writeData(point, intensity, timestamp, num_returns, return_id, rgb);

        return true;
    }

    template <class T>
    bool MultiLasOutputStream<T>::writeData(const double point[3], const uint8_t intensity, const double timestamp,
        const uint8_t num_returns, const uint8_t return_id, const uint8_t rms)
    {
        if (!las_stream_ptr_) return false;
        auto& header = las_stream_ptr_->header_;

        if (max_num_points_ == 0)
            calculateMaxNumPoints();

        if (split_enabled_ && header.number_of_point_records >= max_num_points_ && max_num_points_ > 0)
        {
            if (file_index_ < max_split_allowed_) newLasStream();
            if (!las_stream_ptr_->isOpen()) return false;
        }

        las_stream_ptr_->writeData(point, intensity, timestamp, num_returns, return_id, rms);

        return true;
    }

    template <class T>
    bool MultiLasOutputStream<T>::writeData(const double point[3], const uint8_t intensity, const double timestamp,
        const uint8_t num_returns, const uint8_t return_id, const double azimuth, const uint8_t flight_line_edge)
    {
        if (!las_stream_ptr_) return false;
        auto& header = las_stream_ptr_->header_;

        if (max_num_points_ == 0)
            calculateMaxNumPoints();

        if (split_enabled_ && header.number_of_point_records >= max_num_points_ && max_num_points_ > 0)
        {
            if (file_index_ < max_split_allowed_) newLasStream();
            if (!las_stream_ptr_->isOpen()) return false;
        }

        las_stream_ptr_->writeData(point, intensity, timestamp, num_returns, return_id, azimuth, flight_line_edge);

        return true;
    }

    template <class T>
    void MultiLasOutputStream<T>::calculateMaxNumPoints()
    {
        if (las_stream_ptr_)
        {
            auto& header = las_stream_ptr_->header_;
            max_num_points_ = (static_cast<uint64_t>(max_file_size_) * 1048576 - header.offset_to_point_data) / header.point_data_record_length;
        }
        max_num_points_ = 1 > max_num_points_ ? 1 : max_num_points_;
    }

    template <class T>
    void MultiLasOutputStream<T>::newLasStream()
    {
        ostringstream name_stream;
        name_stream << file_name_.substr(0, file_name_.find_last_of(".")) << "_"
            << setfill('0') << setw(index_padding_) << ++file_index_ << ".las";

        // copy header
        T header_prev = las_stream_ptr_->get_header();
        las_stream_ptr_ = make_shared<LasOutputStream<T>>(name_stream.str());
        auto& header = las_stream_ptr_->header_;
        header = header_prev;

        // reset header
        header.offset_to_point_data = sizeof(header);
        header.number_of_point_records = 0;
        if (header.version_major == 1)
        {
            if (header.version_minor < 4)
                memset(header.number_of_points_by_return, 0, 5 * sizeof(uint32_t));
            else if (header.version_minor == 4)
                memset(header.number_of_points_by_return, 0, 15 * sizeof(uint64_t));
        }
        las_stream_ptr_->writeHeader();
        int i = 0;
        for (auto& h : VLRHeaderBuffer)
        {
            las_stream_ptr_->writeVLR(h, VLRRecordBuffer.data() + i, true);
            i += h.record_length;
        }
    }

    template class MultiLasOutputStream <LegacyLasHeader>;
    template class MultiLasOutputStream <LasHeader>;


    template <class T>
    void getPointData(char point_data[], double& GPS_time, double xyz[3])
    {
        auto point_ptr = reinterpret_cast<T*>(point_data);
        GPS_time = point_ptr->GPS_time;
        xyz[0] = point_ptr->x;
        xyz[1] = point_ptr->y;
        xyz[2] = point_ptr->z;
    }

    template <class T>
    void getPointData(char point_data[], double& GPS_time, double xyz[3], uint16_t& intensity, uint8_t& num_returns, uint8_t& return_id)
    {
        auto point_ptr = reinterpret_cast<T*>(point_data);
        GPS_time = point_ptr->GPS_time;
        xyz[0] = point_ptr->x;
        xyz[1] = point_ptr->y;
        xyz[2] = point_ptr->z;
        intensity = point_ptr->intensity;
        num_returns = point_ptr->num_returns;
        return_id = point_ptr->return_num;
    }

    template <class T>
    void getPointData(char point_data[], double& GPS_time, double xyz[3], uint16_t& intensity, uint8_t& num_returns, uint8_t& return_id, uint16_t rgb[3])
    {
        auto point_ptr = reinterpret_cast<T*>(point_data);
        GPS_time = point_ptr->GPS_time;
        xyz[0] = point_ptr->x;
        xyz[1] = point_ptr->y;
        xyz[2] = point_ptr->z;
        intensity = point_ptr->intensity;
        num_returns = point_ptr->num_returns;
        return_id = point_ptr->return_num;
        rgb[0] = point_ptr->red;
        rgb[1] = point_ptr->green;
        rgb[2] = point_ptr->blue;
    }


    
}

void readlas(string file, pcl::PointCloud<pcl::PointXYZI>::Ptr ret) {
    unique_ptr<las::BaseLasInputStream> las_in_src;
    las_in_src = make_unique<las::LasInputStream<las::LegacyLasHeader>>(file);
    if (!las_in_src->isOpen()) {
        printf("Failed to open file. Error code is %d\n", errno);
        return;
    }
    float version_dest = las_in_src->get_version();
    if (version_dest > 1.35)
    {
        las_in_src.reset(new las::LasInputStream<las::LasHeader>(file));
    }

    las::LasPointType input_point_type_src = las_in_src->get_point_format();

    point_cloud point;
    double GPS_time;
    double xyz[3], newxyz[3];
    uint16_t intensity;
    uint16_t rgb[3] = {};
    uint8_t num_returns, return_id;
    uint8_t classification = 0;
    double scale[3], offset[3];
    char point_data[128];
    double basexyz[3];
    bool flag = true;

    las_in_src->get_scale_factor(scale);
    las_in_src->get_offset(offset);
    bool has_point = las_in_src->get_next_point(point_data);
    while (has_point)
    {
        switch (input_point_type_src)
        {
        case las::LasPointType::LasPoint1:
            las::getPointData<las::LasPoint1>(point_data, GPS_time, xyz, intensity, num_returns, return_id);
            break;
        case las::LasPointType::LasPoint3:
            las::getPointData<las::LasPoint3>(point_data, GPS_time, xyz, intensity, num_returns, return_id);
            break;
        case las::LasPointType::LasPoint6:
            las::getPointData<las::LasPoint6>(point_data, GPS_time, xyz, intensity, num_returns, return_id);
            break;
        case las::LasPointType::LasPoint7:
            las::getPointData<las::LasPoint7>(point_data, GPS_time, xyz, intensity, num_returns, return_id);
            break;
        default:
            break;
        }
        // transform point
        //cout << offset[0] << " " << offset[1] << " " << offset[2] << endl;
        for (int i = 0; i < 3; i++)
        {
            point.xyz[i] = xyz[i] * scale[i] + offset[i];
            if (flag)
            {
                basexyz[i] = point.xyz[i];
            }
            //point.xyz[i] = point.xyz[i] - basexyz[i];
        }

        flag = false;
        point.time_of_week = GPS_time;

        pcl::PointXYZI pcl_point((float)point.xyz[0], (float)point.xyz[1], (float)point.xyz[2], (float)intensity);
        ret->points.push_back(pcl_point);
        ret->height = 1;
        ret->width = ret->points.size();
        has_point = las_in_src->get_next_point(point_data);
        
    }
    return;
}


void readlas(string file, pcl::PointCloud<PointXYZIT>::Ptr ret) {
    unique_ptr<las::BaseLasInputStream> las_in_src;
    las_in_src = make_unique<las::LasInputStream<las::LegacyLasHeader>>(file);
    if (!las_in_src->isOpen()) {
        printf("Failed to open file. Error code is %d\n", errno);
        return;
    }
    float version_dest = las_in_src->get_version();
    if (version_dest > 1.35)
    {
        las_in_src.reset(new las::LasInputStream<las::LasHeader>(file));
    }

    las::LasPointType input_point_type_src = las_in_src->get_point_format();

    point_cloud point;
    double GPS_time;
    double xyz[3], newxyz[3];
    uint16_t intensity;
    uint16_t rgb[3] = {};
    uint8_t num_returns, return_id;
    uint8_t classification = 0;
    double scale[3], offset[3];
    char point_data[128];
    double basexyz[3];
    bool flag = true;

    las_in_src->get_scale_factor(scale);
    las_in_src->get_offset(offset);
    bool has_point = las_in_src->get_next_point(point_data);
    while (has_point)
    {
        switch (input_point_type_src)
        {
        case las::LasPointType::LasPoint1:
            las::getPointData<las::LasPoint1>(point_data, GPS_time, xyz, intensity, num_returns, return_id);
            break;
        case las::LasPointType::LasPoint3:
            las::getPointData<las::LasPoint3>(point_data, GPS_time, xyz, intensity, num_returns, return_id);
            break;
        case las::LasPointType::LasPoint6:
            las::getPointData<las::LasPoint6>(point_data, GPS_time, xyz, intensity, num_returns, return_id);
            break;
        case las::LasPointType::LasPoint7:
            las::getPointData<las::LasPoint7>(point_data, GPS_time, xyz, intensity, num_returns, return_id);
            break;
        default:
            break;
        }
        // transform point
        //cout << offset[0] << " " << offset[1] << " " << offset[2] << endl;
        for (int i = 0; i < 3; i++)
        {
            point.xyz[i] = xyz[i] * scale[i] + offset[i];
            if (flag)
            {
                basexyz[i] = point.xyz[i];
            }
            //point.xyz[i] = point.xyz[i] - basexyz[i];
        }

        flag = false;
        point.time_of_week = GPS_time;

        PointXYZIT pcl_point((float)point.xyz[0], (float)point.xyz[1], (float)point.xyz[2], (float)intensity, GPS_time);
        ret->points.push_back(pcl_point);
        ret->height = 1;
        ret->width = ret->points.size();
        has_point = las_in_src->get_next_point(point_data);

    }
    return;
}