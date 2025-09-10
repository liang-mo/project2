#pragma once
#pragma once

#ifndef BEIDOU_GRID_3D_H_
#define BEIDOU_GRID_3D_H_

#include "PositioningGrid.h"
#include <vector>

namespace Grid
{
    class BeiDouGrid3D final : public PositioningGrid
    {
    public:
        BeiDouGrid3D();
        ~BeiDouGrid3D() override;

        /**
         * @brief ��ʼ������ĳ�ʼ��Ϣ
         */
        void Init() override;

        /**
         * @brief ��ȡ��������
         * @return GridType ��������
         */
        [[nodiscard]] GridType GetGridType() const override;

        /**
         * @brief ��ȡ������������
         * @param gridCode �������
         * @return glm::dvec3 ������������
         */
        [[nodiscard]] glm::dvec3 GetGridCenter(const std::string& gridCode) const override;

        /**
         * @brief ��ȡ������������
         * @param lonlatheight �������
         * @param level ������
         * @return glm::dvec3 ������������
         */
        [[nodiscard]] glm::dvec3 GetGridCenter(const glm::dvec3& lonlatheight, const int32& level) const override;

        /**
         * @brief ��ȡ��ά������������
         * @param gridCode �������
         * @return glm::dvec3 ������������
         */
        [[nodiscard]] virtual glm::dvec3 GetGrid2DCenter(const std::string& gridCode) const override;

        /**
         * @brief ��ȡ��ά������������
         * @param point �������
         * @param level ������
         * @return glm::dvec3 ������������
         */
        [[nodiscard]] virtual glm::dvec3 GetGrid2DCenter(const glm::dvec3& point, const int32& level) const override;

        /**
         * @brief ���������ڵĶ�ά�������
         * @param lonlatheight �������
         * @param level �������񼶱�
         * @return std::string �������
         */
        [[nodiscard]] std::string BuildGridCode2D(const glm::dvec3& lonlatheight, const int32& level) const override;

        /**
         * @brief ���������ڵ���ά�������
         * @param lonlatheight �������
         * @param level �������񼶱�
         * @return std::string �������
         */
        [[nodiscard]] std::string BuildGridCode3D(const glm::dvec3& lonlatheight, const int32& level) const override;

        /**
         * @return ����ָ������������ھ��ȷ���ĸ���
         * @param min ��С����
         * @param max ��󾭶�
         * @param level ���񼶱�
         * @return ���񾭶ȷ���ĸ���
         */
        [[nodiscard]] virtual int64 GetLonMaxGrid(const double& min, const double& max, const int32& level) const override;

        /**
         * @return ����ָ�������������γ�ȷ���ĸ���
         * @param min ��Сγ��
         * @param max ���γ��
         * @param level ���񼶱�
         * @return ����γ�ȷ���ĸ���
         */
        [[nodiscard]] virtual int64 GetLatMaxGrid(const double& min, const double& max, const int32& level) const override;

        /**
         * @return ����ָ������������ڸ߶ȷ���ĸ���
         * @param min ��С�߶�
         * @param max ���߶�
         * @param level ���񼶱�
         * @return ����߶ȷ���ĸ���
         */
        [[nodiscard]] virtual int64 GetHeightMaxGrid(const double& min, const double& max, const int32& level) const override;

        /**
         * @brief ��ȡ��Χ�з�Χ��ָ�������µ���������
         * @param min ��Χ����С�ǵ�
         * @param max ��Χ�����ǵ�
         * @param level ���񼶱�
         * @return ��������
         */
        [[nodiscard]] virtual int64 GetMaxGridNumByBox(const glm::dvec3& min, const glm::dvec3& max, const int32& level) const override;

        /**
         * @brief ��ȡ�������񾭶ȵĿ��
         * @param level ���񼶱�
         * @return �������񾭶ȵĿ��
         */
        [[nodiscard]] virtual double GetStepLon(const int32& level) const override;

        /**
         * @brief ��ȡ��������γ�ȵĿ��
         * @param level ���񼶱�
         * @return ��������γ�ȵĿ��
         */
        [[nodiscard]] virtual double GetStepLat(const int32& level) const override;

        /**
         * @brief ���ݲ�������㱻ռ�õ�ȫ������
         * @param samplePoints ������
         * @param level ���񼶱�
         * @param aggregationLevel ��Ҫ�ۺϵ����񼶱�
         * @param grids ���ɵ����񼯺�
         * @param minHeight ��С�߳�
         * @param maxHeight ���߳�
         */
        void GenOccupiedGridsWithSamplePoints(const std::vector<glm::dvec3>& samplePoints, const int32& level, const int32& aggregationLevel,
            std::map<std::string, std::vector<FGridInfo>>& grids, const double minHeight,
            const double maxHeight) const override;

        /**
         * @brief ���ݶ�ά�����룬�Ͷ�ά�����ڵĲ����㣬���ɵ�ǰ�����µ�ȫ������
         * @param parentCode2D ��ǰ���������ڵĸ�����ά������
         * @param samplePoints ������
         * @param level ʵ�����񼶱�
         * @param minHeight ��С�߳�
         * @param maxHeight ���߳�
         * @param grids ���ɵ����񼯺�
         */
        virtual void GenFullGridsWithParentCode2D(const std::string& parentCode2D, const std::vector<glm::dvec3>& samplePoints, const int32& level,
            const double& minHeight, const double& maxHeight,
            std::map<std::string, std::vector<FGridInfo>>& grids) const override;

        /**
         * @brief ���ݵ�ǰ�߶ȣ���ȡ��Ӧ������������������0��ʼ����
         * @param deltaLatDegree γ�ȼ��
         * @param curHeight ��ǰ�߶�
         * @return glm::int64 ��������
         */
        static [[nodiscard]] glm::int64 GetBeidouGridHeightIndex(const double& deltaLatDegree, const double& curHeight);

        /**
         * @brief ��ȡָ�������ı�������ĵ���߶�
         * @param deltaLatDegree γ�ȼ��
         * @param heightIndex ����������������0��ʼ
         * @return double �������߶�
         */
        static [[nodiscard]] double GetBeiDouGridHeight(const double& deltaLatDegree, const int32& heightIndex);

        /**
         * @brief ��ȡ�������񳤻��߸�
         * @param deltaLatDegree γ�ȼ��
         * @param heightIndex �߶�����
         * @return double ����߶�
         */
        static [[nodiscard]] double GetBeiDouGridZorYHeight(const double& deltaLatDegree, const int32& heightIndex);

        /**
         * @brief ����У��
         * @param lon ����
         * @param lat γ��
         * @param level ����
         * @param outLon �������
         * @param outLat ���γ��
         * @param bUseAnchorCorner �Ƿ�ʹ��ê���
         * @param bExtend �Ƿ���չ
         */
        static void CorrectionCoordinates(const double lon, const double lat, const int32 level, double& outLon, double& outLat,
            const bool bUseAnchorCorner = true, const bool bExtend = false);

    private:
        /**
         * @brief ��ȡ��ͬ�����µľ��Ȳ���
         * @param level �������񼶱�
         * @return ���Ȳ������Ƕ�ֵ
         */
        [[nodiscard]] double GetStepLonDegree(const int32& level) const;

        /**
         * @brief ��ȡ��ͬ�����µ�ά�Ȳ���
         * @param level �������񼶱�
         * @return γ�Ȳ������Ƕ�ֵ
         */
        [[nodiscard]] double GetStepLatDegree(const int32& level) const;

        /**
         * @brief ��ȡ��ͬ�����£����ڼ���γ�ȷ���͸߶ȷ���ģ���γ�Ȳ���
         * @param level �������񼶱�
         * @return ������ȡ���Ȳ�������������ȡγ�Ȳ������Ƕ�ֵ
         */
        [[nodiscard]] double GetStepZorYDegree(const int32& level) const;

        [[nodiscard]] std::vector<std::string> BuildBeiDouGrid2D(const glm::dvec3& lonlatheight, const int32& level) const;
        [[nodiscard]] std::vector<std::string> BuildBeiDouGridHeightCode(const double height, const int32& level) const;
        [[nodiscard]] std::string ConnectBeiDouGridCode(const std::vector<std::string>& code2d, const std::vector<std::string>& codeHeight) const;

        [[nodiscard]] int32 ConvertChar2Int32(const char c) const;

        [[nodiscard]] int64_t IndexOfVector(const std::vector<char>& v, char value) const;

        [[nodiscard]] int32 GetLevelByCode2D(const std::string_view& code2d) const;

        [[nodiscard]] int32 GetLevelByCode3D(const std::string_view& code3d) const;

    private:
        GridType _gridType = GridType::kBeiDouGrid;

        std::vector<double> _levelClassifiedStandardLon;
        std::vector<double> _levelClassifiedStandardLat;

        std::vector<char> _codeLetter;
        std::vector<char> _codeNumber;
    };

}  // namespace Grid

#endif

// end
