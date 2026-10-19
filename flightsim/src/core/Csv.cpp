#include "core/Csv.h"

#include <cstdlib>
#include <fstream>
#include <sstream>

namespace fsim {

bool CsvReader::open(const std::string& path) {
    auto f = std::make_unique<std::ifstream>(path, std::ios::binary);
    if (!*f) return false;
    in_ = std::move(f);
    header_.clear();
    return readRecord(header_);
}

bool CsvReader::openText(std::string text) {
    in_ = std::make_unique<std::istringstream>(std::move(text));
    header_.clear();
    return readRecord(header_);
}

bool CsvReader::readRecord(std::vector<std::string>& out) {
    out.clear();
    std::string line;
    while (in_ && std::getline(*in_, line)) {
        if (!line.empty() && line.back() == '\r') line.pop_back();
        if (line.empty() || line[0] == '#') continue;
        std::string cur;
        bool quoted = false;
        for (size_t i = 0; i < line.size(); ++i) {
            const char c = line[i];
            if (quoted) {
                if (c == '"' && i + 1 < line.size() && line[i + 1] == '"') {
                    cur.push_back('"');
                    ++i;
                } else if (c == '"') {
                    quoted = false;
                } else {
                    cur.push_back(c);
                }
            } else if (c == '"') {
                quoted = true;
            } else if (c == ',') {
                out.push_back(std::move(cur));
                cur.clear();
            } else {
                cur.push_back(c);
            }
        }
        out.push_back(std::move(cur));
        return true;
    }
    return false;
}

bool CsvReader::next() {
    return readRecord(row_);
}

int CsvReader::column(std::string_view name) const {
    for (size_t i = 0; i < header_.size(); ++i)
        if (header_[i] == name) return static_cast<int>(i);
    return -1;
}

std::string_view CsvReader::field(int col) const {
    if (col < 0 || static_cast<size_t>(col) >= row_.size()) return {};
    return row_[static_cast<size_t>(col)];
}

std::string_view CsvReader::field(std::string_view name) const {
    return field(column(name));
}

double CsvReader::getDouble(std::string_view name, double def) const {
    const std::string s(field(name));
    if (s.empty()) return def;
    char* end = nullptr;
    const double d = std::strtod(s.c_str(), &end);
    return end == s.c_str() ? def : d;
}

int CsvReader::getInt(std::string_view name, int def) const {
    const std::string s(field(name));
    if (s.empty()) return def;
    char* end = nullptr;
    const long n = std::strtol(s.c_str(), &end, 10);
    return end == s.c_str() ? def : static_cast<int>(n);
}

// ------------------------------------------------------------------------------------------------

CsvWriter::~CsvWriter() {
    close();
}

bool CsvWriter::open(const std::string& path, const std::vector<std::string>& header) {
    close();
    file_ = std::fopen(path.c_str(), "w");
    if (!file_) return false;
    for (const std::string& h : header) addString(h);
    endRow();
    return true;
}

void CsvWriter::sep() {
    if (rowStarted_) std::fputc(',', file_);
    rowStarted_ = true;
}

void CsvWriter::addDouble(double value, int decimals) {
    if (!file_) return;
    sep();
    std::fprintf(file_, "%.*f", decimals, value);
}

void CsvWriter::addInt(long long value) {
    if (!file_) return;
    sep();
    std::fprintf(file_, "%lld", value);
}

void CsvWriter::addString(std::string_view value) {
    if (!file_) return;
    sep();
    const bool needsQuote = value.find_first_of(",\"\n") != std::string_view::npos;
    if (!needsQuote) {
        std::fwrite(value.data(), 1, value.size(), file_);
        return;
    }
    std::fputc('"', file_);
    for (char c : value) {
        if (c == '"') std::fputc('"', file_);
        std::fputc(c, file_);
    }
    std::fputc('"', file_);
}

void CsvWriter::endRow() {
    if (!file_) return;
    std::fputc('\n', file_);
    rowStarted_ = false;
}

void CsvWriter::close() {
    if (file_) std::fclose(file_);
    file_ = nullptr;
    rowStarted_ = false;
}

}  // namespace fsim
