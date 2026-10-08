#pragma once
#include <chrono>
#include <string>
#include <vector>
#include <unordered_map>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <algorithm>
#include <numeric>

namespace ProfilerSection {
    constexpr const char* PHYSICS           = "Physics (Jolt)";
    constexpr const char* ROVER_UPDATE      = "Rover Update";
    constexpr const char* CHUNK_STREAM      = "Chunk Streaming";
    constexpr const char* NAV_DISCOVERY     = "NavGraph Discovery";
    constexpr const char* MOUSE_PICK        = "Mouse Ray Pick";
    constexpr const char* RENDER_SKY_SUN    = "Render: Sky & Sun";
    constexpr const char* RENDER_BASE_LAND  = "Render: Base 1.44km Terrain";
    constexpr const char* RENDER_CHUNKS     = "Render: Chunks Grid";
    constexpr const char* RENDER_BOULDERS   = "Render: Boulders";
    constexpr const char* RENDER_EDGES      = "Render: NavGraph Edges";
    constexpr const char* RENDER_NODES      = "Render: NavGraph Nodes";
    constexpr const char* RENDER_DIJKSTRA   = "Render: Dijkstra & Path";
    constexpr const char* RENDER_ROVER      = "Render: Rover Model";
    constexpr const char* RENDER_HUD        = "Render: 2D HUD";
    constexpr const char* GL_SWAP_BUFFERS   = "Render: EndDrawing / VSync";
}

struct FrameProfileEntry {
    uint64_t frameIndex = 0;
    float timeStampSec = 0.0f;
    float totalFrameMs = 0.0f;
    std::unordered_map<std::string, float> sectionTimesMs;
};

class FrameProfiler {
public:
    static FrameProfiler& instance() {
        static FrameProfiler s_inst;
        return s_inst;
    }

    void init(const std::string& logFilePath = "perf_profile.log") {
        m_logFilePath = logFilePath;
        m_logFile.open(m_logFilePath, std::ios::out | std::ios::trunc);
        if (m_logFile.is_open()) {
            m_logFile << "================================================================================\n";
            m_logFile << " 3D DRIVING SIMULATOR - HIGH RESOLUTION PERFORMANCE PROFILING LOG\n";
            m_logFile << " Target: 60 FPS (Frame Budget: 16.67 ms)\n";
            m_logFile << "================================================================================\n\n";
            m_logFile << "Format: [Frame #] [Time(s)] [Total(ms)] -> Section Breakdown (ms)\n";
            m_logFile << "--------------------------------------------------------------------------------\n";
        }
        m_appStartTime = std::chrono::high_resolution_clock::now();
        m_currentFrame = 0;
    }

    void beginFrame() {
        m_frameStart = std::chrono::high_resolution_clock::now();
        m_currentEntry = FrameProfileEntry();
        m_currentEntry.frameIndex = m_currentFrame++;
        auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(m_frameStart - m_appStartTime).count();
        m_currentEntry.timeStampSec = static_cast<float>(elapsed) / 1000000.0f;
    }

    void beginSection(const std::string& name) {
        m_sectionStarts[name] = std::chrono::high_resolution_clock::now();
    }

    void endSection(const std::string& name) {
        auto end = std::chrono::high_resolution_clock::now();
        auto it = m_sectionStarts.find(name);
        if (it != m_sectionStarts.end()) {
            float ms = std::chrono::duration_cast<std::chrono::microseconds>(end - it->second).count() / 1000.0f;
            m_currentEntry.sectionTimesMs[name] = ms;
        }
    }

    void endFrame() {
        auto frameEnd = std::chrono::high_resolution_clock::now();
        m_currentEntry.totalFrameMs = std::chrono::duration_cast<std::chrono::microseconds>(frameEnd - m_frameStart).count() / 1000.0f;

        // Log frame details if it hitches (frame > 16.67ms or any sub-section > 3.0ms)
        bool isHitch = (m_currentEntry.totalFrameMs > 17.5f);
        for (const auto& kv : m_currentEntry.sectionTimesMs) {
            if (kv.second > 3.5f && kv.first != ProfilerSection::GL_SWAP_BUFFERS) {
                isHitch = true;
                break;
            }
        }

        if (isHitch && m_logFile.is_open()) {
            m_logFile << "[HITCH F#" << std::setw(5) << m_currentEntry.frameIndex << "] "
                      << std::fixed << std::setprecision(2)
                      << "t=" << std::setw(6) << m_currentEntry.timeStampSec << "s | "
                      << "Total: " << std::setw(6) << m_currentEntry.totalFrameMs << "ms";

            // Print top contributors
            for (const auto& kv : m_currentEntry.sectionTimesMs) {
                if (kv.second > 0.5f) {
                    m_logFile << " | " << kv.first << ": " << std::setprecision(2) << kv.second << "ms";
                }
            }
            m_logFile << "\n";
            m_logFile.flush();
        }

        m_history.push_back(m_currentEntry);
    }

    void finishAndDumpSummary() {
        if (!m_logFile.is_open()) return;

        m_logFile << "\n\n================================================================================\n";
        m_logFile << " COMPREHENSIVE PERFORMANCE ANALYSIS & BOTTLENECK REPORT\n";
        m_logFile << "================================================================================\n";

        if (m_history.empty()) {
            m_logFile << "No frames recorded.\n";
            m_logFile.close();
            return;
        }

        size_t totalFrames = m_history.size();
        std::vector<float> totalTimes;
        totalTimes.reserve(totalFrames);

        std::unordered_map<std::string, std::vector<float>> sectionData;
        int hitches17ms = 0;
        int hitches33ms = 0;

        for (const auto& f : m_history) {
            totalTimes.push_back(f.totalFrameMs);
            if (f.totalFrameMs > 16.67f) hitches17ms++;
            if (f.totalFrameMs > 33.33f) hitches33ms++;

            for (const auto& kv : f.sectionTimesMs) {
                sectionData[kv.first].push_back(kv.second);
            }
        }

        std::sort(totalTimes.begin(), totalTimes.end());
        float sumTime = std::accumulate(totalTimes.begin(), totalTimes.end(), 0.0f);
        float avgTime = sumTime / totalFrames;
        float medianTime = totalTimes[totalFrames / 2];
        float p95Time = totalTimes[static_cast<size_t>(totalFrames * 0.95)];
        float p99Time = totalTimes[static_cast<size_t>(totalFrames * 0.99)];
        float maxTime = totalTimes.back();
        float avgFPS = avgTime > 0.001f ? (1000.0f / avgTime) : 0.0f;
        float onePercentLowFPS = p99Time > 0.001f ? (1000.0f / p99Time) : 0.0f;

        m_logFile << "Recorded Frames     : " << totalFrames << "\n";
        m_logFile << "Average Frame Time  : " << std::fixed << std::setprecision(2) << avgTime << " ms (" << avgFPS << " FPS)\n";
        m_logFile << "Median Frame Time   : " << medianTime << " ms\n";
        m_logFile << "95th Percentile     : " << p95Time << " ms\n";
        m_logFile << "99th Percentile     : " << p99Time << " ms (1% Low: " << onePercentLowFPS << " FPS)\n";
        m_logFile << "Maximum Frame Hitch : " << maxTime << " ms\n";
        m_logFile << "Frames > 16.67ms    : " << hitches17ms << " (" << std::setprecision(1) << (100.0f * hitches17ms / totalFrames) << "%)\n";
        m_logFile << "Frames > 33.33ms    : " << hitches33ms << " (" << std::setprecision(1) << (100.0f * hitches33ms / totalFrames) << "%)\n";
        m_logFile << "--------------------------------------------------------------------------------\n\n";

        m_logFile << "SUBSYSTEM TIMING BREAKDOWN (Sorted by Worst Maximum Spike):\n";
        m_logFile << std::left << std::setw(30) << "Subsystem Name"
                  << std::right << std::setw(10) << "Avg (ms)"
                  << std::setw(10) << "P95 (ms)"
                  << std::setw(10) << "P99 (ms)"
                  << std::setw(10) << "Max (ms)"
                  << std::setw(12) << "Spikes >3ms\n";
        m_logFile << "--------------------------------------------------------------------------------\n";

        struct SubsystemStat {
            std::string name;
            float avg;
            float p95;
            float p99;
            float max;
            int countOver3ms;
        };
        std::vector<SubsystemStat> stats;

        for (auto& kv : sectionData) {
            auto& vals = kv.second;
            std::sort(vals.begin(), vals.end());
            float sum = std::accumulate(vals.begin(), vals.end(), 0.0f);
            float avg = sum / totalFrames; // averaged over all frames
            float p95 = vals[static_cast<size_t>(vals.size() * 0.95)];
            float p99 = vals[static_cast<size_t>(vals.size() * 0.99)];
            float max = vals.back();
            int over3 = 0;
            for (float v : vals) if (v > 3.0f) over3++;
            stats.push_back({ kv.first, avg, p95, p99, max, over3 });
        }

        std::sort(stats.begin(), stats.end(), [](const SubsystemStat& a, const SubsystemStat& b) {
            return a.max > b.max;
        });

        for (const auto& s : stats) {
            m_logFile << std::left << std::setw(30) << s.name
                      << std::right << std::setw(10) << std::fixed << std::setprecision(2) << s.avg
                      << std::setw(10) << s.p95
                      << std::setw(10) << s.p99
                      << std::setw(10) << s.max
                      << std::setw(12) << s.countOver3ms << "\n";
        }

        m_logFile << "--------------------------------------------------------------------------------\n\n";

        // Top 15 Worst Individual Frames
        m_logFile << "TOP 15 WORST INDIVIDUAL FRAMES:\n";
        auto worstFrames = m_history;
        std::sort(worstFrames.begin(), worstFrames.end(), [](const FrameProfileEntry& a, const FrameProfileEntry& b) {
            return a.totalFrameMs > b.totalFrameMs;
        });

        for (size_t i = 0; i < std::min<size_t>(worstFrames.size(), 15); ++i) {
            const auto& wf = worstFrames[i];
            m_logFile << "#" << std::setw(2) << (i + 1)
                      << " Frame " << std::setw(5) << wf.frameIndex
                      << " at t=" << std::setw(5) << std::setprecision(2) << wf.timeStampSec << "s: "
                      << std::setw(6) << wf.totalFrameMs << " ms -> ";

            std::vector<std::pair<std::string, float>> sortedSections(wf.sectionTimesMs.begin(), wf.sectionTimesMs.end());
            std::sort(sortedSections.begin(), sortedSections.end(), [](const auto& a, const auto& b) {
                return a.second > b.second;
            });
            for (size_t k = 0; k < std::min<size_t>(sortedSections.size(), 4); ++k) {
                if (sortedSections[k].second > 0.3f) {
                    m_logFile << "[" << sortedSections[k].first << ": " << sortedSections[k].second << "ms] ";
                }
            }
            m_logFile << "\n";
        }

        m_logFile << "================================================================================\n";
        m_logFile.close();

        std::cout << "\n=======================================================\n";
        std::cout << " [PROFILER] Performance log written to: " << m_logFilePath << "\n";
        std::cout << " [PROFILER] Total Frames: " << totalFrames
                  << " | Avg FPS: " << std::fixed << std::setprecision(1) << avgFPS
                  << " | 1% Low: " << onePercentLowFPS << " FPS\n";
        std::cout << "=======================================================\n\n";
    }

private:
    FrameProfiler() = default;

    std::string m_logFilePath = "perf_profile.log";
    std::ofstream m_logFile;
    std::chrono::high_resolution_clock::time_point m_appStartTime;
    std::chrono::high_resolution_clock::time_point m_frameStart;
    std::unordered_map<std::string, std::chrono::high_resolution_clock::time_point> m_sectionStarts;
    FrameProfileEntry m_currentEntry;
    std::vector<FrameProfileEntry> m_history;
    uint64_t m_currentFrame = 0;
};

// RAII helper to profile a section scope
class ProfileScope {
public:
    ProfileScope(const std::string& name) : m_name(name) {
        FrameProfiler::instance().beginSection(m_name);
    }
    ~ProfileScope() {
        FrameProfiler::instance().endSection(m_name);
    }
private:
    std::string m_name;
};
