#ifndef COLLISION_PAIR_MANAGER_HPP
#define COLLISION_PAIR_MANAGER_HPP

#include <Eigen/Core>
#include <string>
#include <vector>
#include <map>
#include <memory>

#ifdef USE_PINOCCHIO
#include <pinocchio/multibody/fwd.hpp>
#endif

namespace OSCBF {

/**
 * @brief 碰撞对管理器 - 管理URDF加载、碰撞对设置和距离计算
 * 
 * 功能：
 * 1. 从URDF文件加载机器人模型和几何模型
 * 2. 选择连杆对并生成碰撞对（支持球体化模型）
 * 3. 根据关节角度更新几何位置
 * 4. 计算所有碰撞对的距离
 * 
 * 使用示例：
 * @code
 * CollisionPairManager manager;
 * manager.loadFromUrdf("/path/to/robot.urdf", {"/path/to/meshes"});
 * manager.setCollisionPairs({{"link2", "link3"}, {"link4", "link5"}});
 * 
 * Eigen::VectorXd q(6);
 * manager.updateGeometry(q);
 * auto min_pair = manager.getMinDistancePair();
 * @endcode
 */
class CollisionPairManager {
public:
    /**
     * @brief 碰撞对信息结构
     */
    struct CollisionPairInfo {
        std::string link1_name;      // 连杆1名称
        std::string link2_name;      // 连杆2名称
        size_t geom1_idx;            // 几何对象1索引
        size_t geom2_idx;            // 几何对象2索引
        double distance;              // 当前距离
        Eigen::Vector3d point1;      // 最近点1（在几何对象1上）
        Eigen::Vector3d point2;      // 最近点2（在几何对象2上）
        bool is_colliding;            // 是否碰撞
    };

    /**
     * @brief 构造函数
     */
    CollisionPairManager();

    /**
     * @brief 析构函数
     */
    ~CollisionPairManager();

    CollisionPairManager(CollisionPairManager&&) noexcept;
    CollisionPairManager& operator=(CollisionPairManager&&) noexcept;

    /**
     * @brief 从URDF文件加载机器人模型和几何模型
     * 
     * @param urdf_path URDF文件路径
     * @param package_dirs 包目录列表（用于查找mesh文件），可以为空
     * @return true 如果加载成功，false 否则
     * @throws std::runtime_error 如果URDF文件不存在或加载失败
     */
    bool loadFromUrdf(
        const std::string& urdf_path,
        const std::vector<std::string>& package_dirs = {}
    );

    /**
     * @brief 设置碰撞对（仅对选中的连杆对生成碰撞对）
     * 
     * 对于每个选中的连杆对 (link1, link2)，会生成所有几何对象的组合碰撞对。
     * 例如，如果 link1 有 3 个球体，link2 有 2 个球体，会生成 3*2=6 个碰撞对。
     * 
     * @param link_pairs 连杆对列表，每个元素是 (link1_name, link2_name)
     * @return true 如果设置成功，false 否则
     * @throws std::runtime_error 如果模型未加载或连杆不存在
     */
    bool setCollisionPairs(
        const std::vector<std::pair<std::string, std::string>>& link_pairs
    );

    /**
     * @brief 根据关节角度更新几何位置
     * 
     * 必须先调用 loadFromUrdf() 和 setCollisionPairs()
     * 
     * @param q 关节角度向量
     * @throws std::runtime_error 如果模型未加载或尺寸不匹配
     */
    void updateGeometry(const Eigen::VectorXd& q);

    /**
     * @brief 计算所有碰撞对的距离
     * 
     * 必须先调用 updateGeometry()
     * 
     * @return true 如果计算成功，false 否则
     */
    bool computeDistances();

    /**
     * @brief 获取最小距离的碰撞对信息
     * 
     * @return 最小距离的碰撞对信息，如果没有碰撞对则返回空结构
     */
    CollisionPairInfo getMinDistancePair() const;

    /**
     * @brief 获取所有碰撞对的距离信息
     * 
     * @return 所有碰撞对的信息列表
     */
    std::vector<CollisionPairInfo> getAllCollisionPairs() const;

    /**
     * @brief 获取指定索引的碰撞对信息
     * 
     * @param pair_idx 碰撞对索引
     * @return 碰撞对信息
     * @throws std::out_of_range 如果索引超出范围
     */
    CollisionPairInfo getCollisionPair(size_t pair_idx) const;

    /**
     * @brief 获取碰撞对数量
     * 
     * @return 碰撞对数量
     */
    size_t getNumCollisionPairs() const;

    /**
     * @brief 获取连杆名称列表
     * 
     * @return 所有连杆名称列表
     */
    std::vector<std::string> getLinkNames() const;

    /**
     * @brief 检查模型是否已加载
     * 
     * @return true 如果模型已加载，false 否则
     */
    bool isModelLoaded() const;

    /**
     * @brief 检查碰撞对是否已设置
     * 
     * @return true 如果碰撞对已设置，false 否则
     */
    bool areCollisionPairsSet() const;

    /**
     * @brief 获取关节数量
     * 
     * @return 关节数量
     */
    int getNumJoints() const;

    /**
     * @brief 获取URDF路径
     * 
     * @return URDF文件路径
     */
    std::string getUrdfPath() const;

    /**
     * @brief 获取 Pinocchio Model（用于计算雅可比等）
     * 
     * @return Pinocchio Model 指针，如果未加载则返回 nullptr
     */
#ifdef USE_PINOCCHIO
    pinocchio::Model* getModel() const;
    pinocchio::Data* getData() const;
    pinocchio::GeometryModel* getGeometryModel() const;
    pinocchio::GeometryData* getGeometryData() const;
#endif

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;

#ifdef USE_PINOCCHIO
    static std::map<std::string, std::vector<size_t>> buildLinkToGeomsMap(const Impl& impl);
#endif
    static CollisionPairInfo extractCollisionPairInfo(const Impl& impl, size_t pair_idx);
};

} // namespace OSCBF

#endif // COLLISION_PAIR_MANAGER_HPP
