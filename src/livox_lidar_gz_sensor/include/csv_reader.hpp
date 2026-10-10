/*
 * SPDX-FileCopyrightText: 2021 livox
 * SPDX-FileCopyrightText: 2026 FZ_XiaoXi
 * SPDX-License-Identifier: MIT
 *
 * Derived from Livox laser simulation and modified for strict CSV validation.
 */

#ifndef CSV_READER_HPP_
#define CSV_READER_HPP_

#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

class CsvReader {
 public:
    static bool ReadCsvFile(const std::string &file_name,
                            std::vector<std::vector<double>> &datas) {
        std::ifstream file_stream(file_name);
        if (!file_stream.is_open()) {
            std::cerr << "cannot read csv file!" << file_name << "\n";
            return false;
        }

        std::string line;
        if (!std::getline(file_stream, line)) {
            std::cerr << "csv file is empty: " << file_name << "\n";
            return false;
        }

        std::size_t line_number = 1;
        while (std::getline(file_stream, line)) {
            ++line_number;

            if (line.find_first_not_of(" \t\r\n") == std::string::npos)
                continue;

            std::stringstream line_stream(line);
            std::vector<double> data;
            std::string value;
            bool valid = true;

            while (std::getline(line_stream, value, ',')) {
                try {
                    std::size_t parsed = 0;
                    const double number = std::stod(value, &parsed);
                    if (value.find_first_not_of(" \t\r\n", parsed) !=
                        std::string::npos) {
                        valid = false;
                        break;
                    }
                    data.push_back(number);
                } catch (const std::exception &) {
                    valid = false;
                    break;
                }
            }

            if (!valid || data.size() != 3) {
                std::cerr << "cannot parse CSV line " << line_number
                          << ": " << line << "\n";
                return false;
            }

            datas.push_back(std::move(data));
        }

        return true;
    }
};

#endif  // CSV_READER_HPP_
