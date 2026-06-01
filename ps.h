#ifndef PS_H
#define PS_H

#include "ds.h"

#include <fstream>
#include <sstream>
#include <cctype>
#include <stack>

namespace skew{
    namespace parser {

    /*============================================================
    * Utility
    *===========================================================*/

    inline std::string trim(const std::string& s)
    {
        int l = 0;
        int r = static_cast<int>(s.size()) - 1;

        while (l <= r && std::isspace(s[l]))
            ++l;

        while (r >= l && std::isspace(s[r]))
            --r;

        return (l <= r)
            ? s.substr(l, r - l + 1)
            : "";
    }

    inline int parseLevel(const std::string& line)
    {
        int l = static_cast<int>(line.find('['));
        int r = static_cast<int>(line.find(']'));

        if (l == std::string::npos ||
            r == std::string::npos)
        {
            throw std::runtime_error(
                "Invalid level format: " + line);
        }

        return std::stoi(
            line.substr(l + 1, r - l - 1));
    }

    /*============================================================
    * Parse Clock Tree
    *===========================================================*/

    inline void parseTree(
        const std::string& filename,
        DesignDB& db)
    {
        std::ifstream fin(filename);

        if (!fin)
        {
            throw std::runtime_error(
                "Cannot open file: " + filename);
        }

        std::string line;

        //------------------------------------
        // Root
        //------------------------------------

        if (!std::getline(fin, line))
        {
            throw std::runtime_error(
                "Empty clock tree file.");
        }

        std::string rootName =
            trim(line.substr(
                line.find(':') + 1));

        int rootId =
            db.addTreeNode(
                rootName,
                NodeType::ROOT,
                "ROOT"
            );

        std::stack<int> st;
        st.push(rootId);

        //------------------------------------
        // Remaining nodes
        //------------------------------------

        while (std::getline(fin, line))
        {
            if (trim(line).empty())
                continue;

            int level = parseLevel(line);

            int pos1 =
                static_cast<int>(line.find(']')) + 2;

            int pos2 =
                static_cast<int>(
                    line.find('(', pos1));

            std::string name =
                trim(line.substr(
                    pos1,
                    pos2 - pos1));

            int pos3 =
                static_cast<int>(line.find('('));

            int pos4 =
                static_cast<int>(line.find(')'));

            std::string instType =
                trim(line.substr(
                    pos3 + 1,
                    pos4 - pos3 - 1));

            bool isSink =
                line.find("SINK")
                != std::string::npos;

            NodeType nodeType;

            if (instType == "FIFO")
            {
                nodeType = NodeType::FF;
            }
            else
            {
                nodeType = NodeType::BUFFER;
            }

            int nodeId =
                db.addTreeNode(
                    name,
                    nodeType,
                    instType,
                    level,
                    INVALID_ID,
                    isSink
                );

            while (
                static_cast<int>(st.size())
                > level)
            {
                st.pop();
            }

            int parentId = st.top();

            db.setParent(
                nodeId,
                parentId);

            st.push(nodeId);
        }
    }

    /*============================================================
    * Parse Buffer Library
    *===========================================================*/

    inline void parseLib(
        const std::string& filename,
        DesignDB& db)
    {
        std::ifstream fin(filename);

        if (!fin)
        {
            throw std::runtime_error(
                "Cannot open file: " + filename);
        }

        std::string line;

        CellLib curCell;
        bool insideCell = false;

        while (std::getline(fin, line))
        {
            line = trim(line);

            if (line.empty())
                continue;

            //--------------------------------
            // cell(...)
            //--------------------------------

            if (line.find("cell") !=
                std::string::npos)
            {
                int l =
                    static_cast<int>(
                        line.find('('));

                int r =
                    static_cast<int>(
                        line.find(')'));

                curCell = CellLib();

                curCell.name =
                    trim(line.substr(
                        l + 1,
                        r - l - 1));

                insideCell = true;
            }

            //--------------------------------
            // SIZE w BY h
            //--------------------------------

            else if (
                line.find("SIZE")
                != std::string::npos)
            {
                std::stringstream ss(line);

                std::string tmp;

                ss >> tmp;
                ss >> curCell.width;

                ss >> tmp;

                ss >> curCell.height;

                curCell.updateArea();
            }

            //--------------------------------
            // SS_DELAY
            //--------------------------------

            else if (
                line.find("SS_DELAY")
                != std::string::npos)
            {
                std::stringstream ss(
                    line.substr(
                        line.find(
                            "SS_DELAY")
                        + 8));

                double d;

                while (ss >> d)
                {
                    curCell.ssDelay.push_back(d);
                }
            }

            //--------------------------------
            // FF_DELAY
            //--------------------------------

            else if (
                line.find("FF_DELAY")
                != std::string::npos)
            {
                std::stringstream ss(
                    line.substr(
                        line.find(
                            "FF_DELAY")
                        + 8));

                double d;

                while (ss >> d)
                {
                    curCell.ffDelay.push_back(d);
                }
            }

            //--------------------------------
            // end cell
            //--------------------------------

            else if (
                insideCell &&
                line.find('}')
                != std::string::npos)
            {
                db.addCell(curCell);
                insideCell = false;
            }
        }
    }

    /*============================================================
    * Parse Timing Report
    *===========================================================*/

    inline void parseRpt(
        const std::string& filename,
        Corner corner,
        DesignDB& db)
    {
        std::ifstream fin(filename);

        if (!fin)
        {
            throw std::runtime_error(
                "Cannot open file: " + filename);
        }

        std::string line;

        //------------------------------------
        // Clock Period
        //------------------------------------

        if (std::getline(fin, line))
        {
            int pos =
                static_cast<int>(
                    line.find(':'));

            if (pos != std::string::npos)
            {
                std::string value =
                    trim(
                        line.substr(pos + 1));

                std::stringstream ss(value);

                ss >> db.clockPeriod;
            }
        }

        //------------------------------------
        // Path
        //------------------------------------

        while (std::getline(fin, line))
        {
            if (line.find("Path") != 0)
                continue;

            int colon =
                static_cast<int>(
                    line.find(':'));

            std::string rest =
                trim(
                    line.substr(colon + 1));

            std::stringstream ss(rest);

            std::string launchFF;
            std::string arrow;
            std::string captureFF;

            double delay;

            ss >> launchFF
            >> arrow
            >> captureFF
            >> delay;

            std::string pathName =
                launchFF +
                "->" +
                captureFF;

            if (corner == Corner::SS)
            {
                db.setPathSSDelay(
                    pathName,
                    launchFF,
                    captureFF,
                    delay);
            }
            else
            {
                db.setPathFFDelay(
                    pathName,
                    launchFF,
                    captureFF,
                    delay);
            }
        }
    }

    /*============================================================
    * Parse Everything
    *===========================================================*/

    inline void buildDesignDB(
        const std::string& testcaseDir,
        DesignDB& db)
    {
        std::string treeFile = testcaseDir + "/clk_tree.structure";
        std::string libFile = testcaseDir + "/buf.lib";
        std::string ssRpt = testcaseDir + "/SS_delay.rpt";
        std::string ffRpt = testcaseDir + "/FF_delay.rpt";

        parseLib(libFile, db);

        parseTree(treeFile, db);

        parseRpt(
            ssRpt,
            Corner::SS,
            db);

        parseRpt(
            ffRpt,
            Corner::FF,
            db);

        db.finalizeAfterParsing();
    }

    } // namespace parser
}// namespace skew

#endif