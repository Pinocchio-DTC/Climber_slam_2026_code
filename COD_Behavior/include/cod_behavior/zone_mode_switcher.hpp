#pragma once

#include <algorithm>
#include <cctype>
#include <fstream>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

namespace cod_behavior
{

enum class ZoneMode : int
{
    ATTACK = 1,
    MOVE = 2,
    DEFENSE = 3,
};

struct RectZone
{
    double x_min{0.0};
    double x_max{0.0};
    double y_min{0.0};
    double y_max{0.0};

    bool contains(double x, double y) const
    {
        return x >= x_min && x <= x_max && y >= y_min && y <= y_max;
    }
};

struct ModeZone
{
    ZoneMode mode{ZoneMode::MOVE};
    RectZone rect;
    int priority{0};
    std::size_t order{0};
};

inline std::string trim(const std::string &value)
{
    const auto start = value.find_first_not_of(" \t\r\n");
    if (start == std::string::npos)
    {
        return "";
    }
    const auto end = value.find_last_not_of(" \t\r\n");
    return value.substr(start, end - start + 1);
}

inline std::string toLower(std::string value)
{
    std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c) {
        return static_cast<char>(std::tolower(c));
    });
    return value;
}

inline std::optional<ZoneMode> parseZoneMode(const std::string &mode_name)
{
    const std::string mode_str = toLower(trim(mode_name));
    if (mode_str == "attack")
    {
        return ZoneMode::ATTACK;
    }
    if (mode_str == "move")
    {
        return ZoneMode::MOVE;
    }
    if (mode_str == "defense")
    {
        return ZoneMode::DEFENSE;
    }
    return std::nullopt;
}

inline const char *zoneModeToString(ZoneMode mode)
{
    switch (mode)
    {
    case ZoneMode::ATTACK:
        return "attack";
    case ZoneMode::MOVE:
        return "move";
    case ZoneMode::DEFENSE:
        return "defense";
    default:
        return "unknown";
    }
}

inline int defaultPriorityForMode(ZoneMode mode)
{
    switch (mode)
    {
    case ZoneMode::DEFENSE:
        return 300;
    case ZoneMode::ATTACK:
        return 100;
    case ZoneMode::MOVE:
    default:
        return 0;
    }
}

inline double rectArea(const RectZone &rect)
{
    return std::max(0.0, rect.x_max - rect.x_min) * std::max(0.0, rect.y_max - rect.y_min);
}

inline bool loadModeZonesFromCsv(
    const std::string &csv_path,
    std::vector<ModeZone> &zones,
    std::string *error = nullptr)
{
    zones.clear();

    std::ifstream input(csv_path);
    if (!input.is_open())
    {
        if (error != nullptr)
        {
            *error = "failed to open csv";
        }
        return false;
    }

    std::string line;
    std::size_t line_no = 0;
    while (std::getline(input, line))
    {
        ++line_no;
        line = trim(line);
        if (line.empty() || line[0] == '#')
        {
            continue;
        }

        if (line_no == 1 && toLower(line).find("mode") != std::string::npos)
        {
            continue;
        }

        std::stringstream ss(line);
        std::string mode_str;
        std::string x_min_str;
        std::string x_max_str;
        std::string y_min_str;
        std::string y_max_str;
        std::string priority_str;

        if (!std::getline(ss, mode_str, ',') ||
            !std::getline(ss, x_min_str, ',') ||
            !std::getline(ss, x_max_str, ',') ||
            !std::getline(ss, y_min_str, ',') ||
            !std::getline(ss, y_max_str, ','))
        {
            continue;
        }

        const auto maybe_mode = parseZoneMode(mode_str);
        if (!maybe_mode.has_value())
        {
            continue;
        }

        try
        {
            ModeZone zone;
            zone.mode = maybe_mode.value();
            zone.rect.x_min = std::stod(trim(x_min_str));
            zone.rect.x_max = std::stod(trim(x_max_str));
            zone.rect.y_min = std::stod(trim(y_min_str));
            zone.rect.y_max = std::stod(trim(y_max_str));
            zone.priority = defaultPriorityForMode(zone.mode);
            zone.order = zones.size();

            if (std::getline(ss, priority_str, ','))
            {
                const std::string p = trim(priority_str);
                if (!p.empty())
                {
                    zone.priority = std::stoi(p);
                }
            }

            if (zone.rect.x_min > zone.rect.x_max)
            {
                std::swap(zone.rect.x_min, zone.rect.x_max);
            }
            if (zone.rect.y_min > zone.rect.y_max)
            {
                std::swap(zone.rect.y_min, zone.rect.y_max);
            }

            zones.push_back(zone);
        }
        catch (const std::exception &)
        {
            continue;
        }
    }

    if (zones.empty() && error != nullptr)
    {
        *error = "csv parsed but no valid zones found";
    }
    return !zones.empty();
}

inline std::optional<ZoneMode> resolveZoneMode(
    const std::vector<ModeZone> &zones,
    double x,
    double y)
{
    const ModeZone *best_zone = nullptr;

    for (const auto &zone : zones)
    {
        if (!zone.rect.contains(x, y))
        {
            continue;
        }

        if (best_zone == nullptr)
        {
            best_zone = &zone;
            continue;
        }

        if (zone.priority > best_zone->priority)
        {
            best_zone = &zone;
            continue;
        }

        if (zone.priority == best_zone->priority)
        {
            const double candidate_area = rectArea(zone.rect);
            const double current_area = rectArea(best_zone->rect);

            if (candidate_area < current_area)
            {
                best_zone = &zone;
                continue;
            }

            if (candidate_area == current_area && zone.order > best_zone->order)
            {
                best_zone = &zone;
            }
        }
    }

    if (best_zone == nullptr)
    {
        return std::nullopt;
    }
    return best_zone->mode;
}

}  // namespace cod_behavior
