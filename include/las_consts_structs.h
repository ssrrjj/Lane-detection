#pragma once

#include <cstdint>

#ifndef ENUM_AS_INT
#define ENUM_AS_INT
#include <type_traits>
template <typename Enumeration>
constexpr auto as_int(Enumeration const value)
-> typename std::underlying_type<Enumeration>::type
{
    static_assert(std::is_enum<Enumeration>::value, "parameter is not of type enum or enum class");
    return static_cast<typename std::underlying_type<Enumeration>::type>(value);
}
#endif

namespace las
{
    const static uint16_t LAS_INTENSITY_SCALE = 65535 / 255;
    const static float LAS_SCAN_ANGLE_1_4 = 0.006f;
    const static uint8_t LATEST_VER_MAJOR = 1, LATEST_VER_MINOR = 4;

    enum class LasPointType {
        LasPoint0 = 0,
        LasPoint1 = 1,
        LasPoint3 = 3,
        LasPoint6 = 6,
        LasPoint7 = 7
    };

    enum class CRSFormat : uint8_t
    {
        GeoTiff = 0,
        WKT = 1
    };

    enum class CRSCoordinate : uint8_t
    {
        LidarFrame = 0,
        ECEF = 1,
        UTM = 2,
        ENU = 3,
        LaLnA = 4,
        LnLaA = 5
    };

    // ============ VLR USER ID ==============

    const static char* CRS = "CRS";

    // ============ VLR Record ID ============

    // user ID: CRS
    const static uint16_t CRS_ALL = 1;

    // =======================================

#pragma pack(1)

    struct LAS_CRS
    {
        uint8_t coordinate;
        union
        {
            struct
            {
                double lat;
                double lon;
                double alt;
            } enu_lla_origin;
            struct
            {
                uint8_t zone;
                bool is_south;
            } utm_zone;
        } info;
    };

    struct LegacyLasHeader
    {
        char     file_signature[4];
        uint16_t file_source_ID;
        uint16_t global_encoding;
        uint32_t project_ID_GUID_data_1;
        uint16_t project_ID_GUID_data_2;
        uint16_t project_ID_GUID_data_3;
        uint8_t  project_ID_GUID_data_4[8];
        uint8_t  version_major;
        uint8_t  version_minor;
        char     system_identifier[32];
        char     generating_software[32];
        uint16_t file_creation_day;
        uint16_t file_creation_year;
        uint16_t header_size;
        uint32_t offset_to_point_data;
        uint32_t number_of_variable_length_records;
        uint8_t  point_data_record_format;
        uint16_t point_data_record_length;
        uint32_t number_of_point_records;
        uint32_t number_of_points_by_return[5];
        double   x_scale_factor;
        double   y_scale_factor;
        double   z_scale_factor;
        double   x_offset;
        double   y_offset;
        double   z_offset;
        double   max_x;
        double   min_x;
        double   max_y;
        double   min_y;
        double   max_z;
        double   min_z;
    };

    struct LasHeader
    {
        char     file_signature[4];
        uint16_t file_source_ID;
        uint16_t global_encoding;
        uint32_t project_ID_GUID_data_1;
        uint16_t project_ID_GUID_data_2;
        uint16_t project_ID_GUID_data_3;
        uint8_t  project_ID_GUID_data_4[8];
        uint8_t  version_major;
        uint8_t  version_minor;
        char     system_identifier[32];
        char     generating_software[32];
        uint16_t file_creation_day;
        uint16_t file_creation_year;
        uint16_t header_size;
        uint32_t offset_to_point_data;
        uint32_t number_of_variable_length_records;
        uint8_t  point_data_record_format;
        uint16_t point_data_record_length;
        uint32_t legacy_number_of_point_records;
        uint32_t legacy_number_of_points_by_return[5];
        double   x_scale_factor;
        double   y_scale_factor;
        double   z_scale_factor;
        double   x_offset;
        double   y_offset;
        double   z_offset;
        double   max_x;
        double   min_x;
        double   max_y;
        double   min_y;
        double   max_z;
        double   min_z;
        uint64_t start_of_waveform_data_packet_record;
        uint64_t start_of_first_extended_variable_length_record;
        uint32_t number_of_extended_variable_length_record;
        uint64_t number_of_point_records;
        uint64_t number_of_points_by_return[15];
    };

    struct BaseLasPoint
    {
        int32_t x;
        int32_t y;
        int32_t z;
        uint16_t intensity;
    };

    struct LasPoint0 : public BaseLasPoint//__attribute__((__packed__))
    {
        uint8_t return_num : 3;
        uint8_t num_returns : 3;
        uint8_t scan_dir_flag : 1;
        uint8_t edge_of_flight_line : 1;
        uint8_t classification;
        char scan_angle;
        uint8_t user_data;
        uint16_t point_source_id;
    };

    struct LasPoint1 : public LasPoint0 //__attribute__((__packed__))
    {
        double   GPS_time;
    };

    struct LasPoint3 : public LasPoint1 //__attribute__((__packed__))
    {
        uint16_t red;
        uint16_t green;
        uint16_t blue;
    };

    struct LasPoint6 : public BaseLasPoint //__attribute__((__packed__))
    {
        uint16_t return_num : 4;
        uint16_t num_returns : 4;
        uint16_t classification_flags : 4;
        uint16_t scanner_channel : 2;
        uint16_t scan_dir_flag : 1;
        uint16_t edge_of_flight_line : 1;
        uint8_t  classification;
        uint8_t  user_data;
        int16_t  scan_angle;
        uint16_t point_source_id;
        double   GPS_time;
    };

    struct LasPoint7 : public LasPoint6 //__attribute__((__packed__))
    {
        uint16_t red;
        uint16_t green;
        uint16_t blue;
    };

    // variable length record header
    struct VLRHeader
    {
        uint16_t reserved;
        char user_ID[16];
        uint16_t record_ID;
        uint16_t record_length;
        char description[32];
    };

#pragma pack()
}


