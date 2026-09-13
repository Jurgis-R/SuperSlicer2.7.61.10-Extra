///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

#include "GCodeLineParser.hpp"

#include <charconv>
#include <cmath>
#include <fast_float/fast_float.h>

/*
G-code lexical parsing implementation
=====================================

The parser keeps framing and number recognition independent from all machine
state. A caller may therefore interpret the same line differently for each
firmware while sharing the exact same lexical rules.
*/

namespace slic3r_api { namespace GCodeGeneration {

GCodeLineParser::GCodeLine::GCodeLine()
{
    this->reset();
}

void GCodeLineParser::GCodeLine::reset()
{
    m_raw.clear();
}

std::string_view GCodeLineParser::GCodeLine::command() const
{
    const std::string_view raw(m_raw);
    const size_t begin = GCodeLineParser::skip_whitespaces(raw);
    const size_t end = GCodeLineParser::skip_word(raw, begin);
    return raw.substr(begin, end - begin);
}

std::string_view GCodeLineParser::GCodeLine::comment() const
{
    const size_t position = m_raw.find(';');
    return position == std::string::npos ? std::string_view() :
                                           std::string_view(m_raw).substr(position + 1);
}

std::string_view GCodeLineParser::GCodeLine::parameter_position(char parameter) const
{
    return GCodeLineParser::parameter_position(m_raw, parameter);
}

bool GCodeLineParser::GCodeLine::has_parameter(char parameter) const
{
    return !GCodeLineParser::parameter_position(m_raw, parameter).empty();
}

std::optional<float> GCodeLineParser::GCodeLine::parameter_value(char parameter) const
{
    float value = 0.f;
    return this->parameter_value(parameter, value) ? std::optional<float>(value) : std::nullopt;
}

bool GCodeLineParser::GCodeLine::parameter_value(char parameter, float &value) const
{
    return GCodeLine::parameter_value(this->parameter_position(parameter), value);
}

bool GCodeLineParser::GCodeLine::parameter_value(char parameter, int32_t &value) const
{
    return GCodeLine::parameter_value(this->parameter_position(parameter), value);
}

bool GCodeLineParser::GCodeLine::parameter_value(std::string_view position, float &value)
{
    if (position.empty())
        return false;

    const char *begin = position.data() + 1;
    const char *end = position.data() + position.size();
    while (begin < end && GCodeLineParser::is_whitespace(*begin))
        ++begin;
    float parsed_value = 0.f;
    const fast_float::from_chars_result parsed = fast_float::from_chars(begin, end, parsed_value);
    if (parsed.ptr == begin || parsed.ec != std::errc() || !std::isfinite(parsed_value) ||
        (parsed.ptr != end && !GCodeLineParser::is_end_of_word(*parsed.ptr)))
        return false;
    value = parsed_value;
    return true;
}

bool GCodeLineParser::GCodeLine::parameter_value(std::string_view position, int32_t &value)
{
    if (position.empty())
        return false;

    const char *begin = position.data() + 1;
    const char *end = position.data() + position.size();
    while (begin < end && GCodeLineParser::is_whitespace(*begin))
        ++begin;
    if (begin < end && *begin == '+')
        ++begin;
    int32_t parsed_value = 0;
    const std::from_chars_result parsed = std::from_chars(begin, end, parsed_value);
    if (parsed.ptr == begin || parsed.ec != std::errc() ||
        (parsed.ptr != end && !GCodeLineParser::is_end_of_word(*parsed.ptr)))
        return false;
    value = parsed_value;
    return true;
}

bool GCodeLineParser::GCodeLine::command_is(std::string_view gcode_line,
                                             const char *expected_command)
{
    if (expected_command == nullptr)
        return false;
    size_t command = GCodeLineParser::skip_whitespaces(gcode_line);
    if (command < gcode_line.size() && gcode_line[command] == 'N') {
        command = GCodeLineParser::skip_word(gcode_line, command);
        command = GCodeLineParser::skip_whitespaces(gcode_line, command);
    }
    const std::string_view expected(expected_command);
    if (expected.size() > gcode_line.size() - command ||
        gcode_line.substr(command, expected.size()) != expected)
        return false;
    const size_t after_command = command + expected.size();
    return after_command == gcode_line.size() ||
           GCodeLineParser::is_end_of_word(gcode_line[after_command]);
}

bool GCodeLineParser::GCodeLine::command_starts_with(std::string_view gcode_line,
                                                      const char *expected_command)
{
    if (expected_command == nullptr)
        return false;
    const size_t command = GCodeLineParser::skip_whitespaces(gcode_line);
    const std::string_view expected(expected_command);
    return expected.size() <= gcode_line.size() - command &&
           gcode_line.substr(command, expected.size()) == expected;
}

std::string GCodeLineParser::GCodeLine::extract_command(std::string_view gcode_line)
{
    const size_t begin = GCodeLineParser::skip_whitespaces(gcode_line);
    const size_t end = GCodeLineParser::skip_word(gcode_line, begin);
    return std::string(gcode_line.substr(begin, end - begin));
}

void GCodeLineParser::parse_buffer(std::string_view buffer)
{
    this->parse_buffer(buffer, [](GCodeLineParser &, const GCodeLine &) {});
}

const char *GCodeLineParser::parse_line_text(const char *begin,
                                              const char *end,
                                              GCodeLine &line)
{
    if (begin == nullptr || end == nullptr || begin > end)
        return end;

    // A caller may reuse a line object directly, without going through
    // parse_buffer(), so an empty source line must also replace prior text.
    line.m_raw.clear();
    const char *cursor = begin;
    while (cursor < end && !is_end_of_line(*cursor))
        ++cursor;
    if (cursor > begin)
        line.m_raw.assign(begin, cursor);

    if (cursor < end && *cursor == '\r')
        ++cursor;
    if (cursor < end && *cursor == '\n')
        ++cursor;
    return cursor;
}

bool GCodeLineParser::is_whitespace(char character)
{
    return character == ' ' || character == '\t';
}

bool GCodeLineParser::is_end_of_line(char character)
{
    return character == '\r' || character == '\n' || character == 0;
}

bool GCodeLineParser::is_end_of_gcode_line(char character)
{
    return character == ';' || is_end_of_line(character);
}

bool GCodeLineParser::is_end_of_word(char character)
{
    return is_whitespace(character) || is_end_of_gcode_line(character);
}

size_t GCodeLineParser::skip_whitespaces(std::string_view text, size_t position)
{
    while (position < text.size() && is_whitespace(text[position]))
        ++position;
    return position;
}

size_t GCodeLineParser::skip_word(std::string_view text, size_t position)
{
    while (position < text.size() && !is_end_of_word(text[position]))
        ++position;
    return position;
}

std::string_view GCodeLineParser::parameter_position(std::string_view raw_line,
                                                      char parameter)
{
    size_t cursor = skip_whitespaces(raw_line);
    cursor = skip_word(raw_line, cursor);
    while (cursor < raw_line.size() && !is_end_of_gcode_line(raw_line[cursor])) {
        cursor = skip_whitespaces(raw_line, cursor);
        if (cursor == raw_line.size() || is_end_of_gcode_line(raw_line[cursor]))
            break;
        if (raw_line[cursor] == parameter)
            return raw_line.substr(cursor);
        cursor = skip_word(raw_line, cursor);
    }
    return {};
}

}} // namespace slic3r_api::GCodeGeneration
