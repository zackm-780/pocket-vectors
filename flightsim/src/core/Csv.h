#pragma once
// CSV reading (header-indexed, streaming) and writing. Used for nav data packs and traces.
//
// Reader: first row is the header. Fields may be double-quoted ("a,b", "" is an escaped quote).
// Blank lines and lines starting with '#' are skipped. Rows are read one at a time, so large
// files never sit in memory.

#include <cstdio>
#include <istream>
#include <memory>
#include <string>
#include <string_view>
#include <vector>

namespace fsim {

class CsvReader {
public:
    // Open a file; returns false if it cannot be read or has no header row.
    bool open(const std::string& path);
    // Read from an in-memory string (tests, fixtures).
    bool openText(std::string text);

    // Advance to the next data row. Returns false at end of input.
    bool next();

    // Column index for a header name, or -1.
    int column(std::string_view name) const;
    const std::vector<std::string>& header() const { return header_; }

    // Fields of the current row (valid until the next call to next()).
    std::string_view field(int col) const;
    std::string_view field(std::string_view name) const;
    double getDouble(std::string_view name, double def = 0) const;
    int getInt(std::string_view name, int def = 0) const;

private:
    bool readRecord(std::vector<std::string>& out);

    std::unique_ptr<std::istream> in_;
    std::vector<std::string> header_;
    std::vector<std::string> row_;
};

// Streaming writer. Does no quoting beyond what the caller passes; numbers are written with
// printf-style formats chosen by the caller.
class CsvWriter {
public:
    CsvWriter() = default;
    ~CsvWriter();
    CsvWriter(const CsvWriter&) = delete;
    CsvWriter& operator=(const CsvWriter&) = delete;

    bool open(const std::string& path, const std::vector<std::string>& header);
    bool isOpen() const { return file_ != nullptr; }
    // Begin a row; add fields with the add* calls; end with endRow().
    void addDouble(double value, int decimals = 4);
    void addInt(long long value);
    void addString(std::string_view value);
    void endRow();
    void close();

private:
    void sep();
    std::FILE* file_ = nullptr;
    bool rowStarted_ = false;
};

}  // namespace fsim
