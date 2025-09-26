#!/bin/bash

# 具身双臂机器人系统启动脚本
# 用于一键启动完整的机器人任务执行系统

set -e  # 遇到错误立即退出

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# 日志函数
log_info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

log_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

log_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

log_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# 检查ROS环境
check_ros_environment() {
    log_info "检查ROS环境..."
    
    if [ -z "$ROS_DISTRO" ]; then
        log_error "ROS环境未设置，请先source ROS环境"
        log_info "请运行: source /opt/ros/noetic/setup.bash"
        exit 1
    fi
    
    log_success "ROS环境检查通过: $ROS_DISTRO"
}

# 检查工作空间
check_workspace() {
    log_info "检查工作空间..."
    
    WORKSPACE_PATH="/Users/chloeya/realman_demo"
    
    if [ ! -d "$WORKSPACE_PATH" ]; then
        log_error "工作空间不存在: $WORKSPACE_PATH"
        exit 1
    fi
    
    cd "$WORKSPACE_PATH"
    
    if [ ! -f "devel/setup.bash" ]; then
        log_warning "工作空间未编译，正在编译..."
        compile_workspace
    fi
    
    # Source工作空间
    source devel/setup.bash
    log_success "工作空间检查通过"
}

# 编译工作空间
compile_workspace() {
    log_info "编译工作空间..."
    
    if ! catkin_make; then
        log_error "工作空间编译失败"
        exit 1
    fi
    
    log_success "工作空间编译完成"
}

# 检查依赖包
check_dependencies() {
    log_info "检查依赖包..."
    
    # 检查关键包是否存在
    local missing_packages=()
    
    if ! rospack find task_planning > /dev/null 2>&1; then
        missing_packages+=("task_planning")
    fi
    
    if ! rospack find embodied_arm_driver > /dev/null 2>&1; then
        missing_packages+=("embodied_arm_driver")
    fi
    
    if [ ${#missing_packages[@]} -ne 0 ]; then
        log_error "缺少以下包: ${missing_packages[*]}"
        log_info "请确保所有包都已正确安装"
        exit 1
    fi
    
    log_success "依赖包检查通过"
}

# 启动系统
start_system() {
    log_info "启动机器人系统..."
    
    # 检查参数
    local debug_mode="false"
    local output_mode="screen"
    local start_control="true"
    
    # 解析命令行参数
    while [[ $# -gt 0 ]]; do
        case $1 in
            --debug)
                debug_mode="true"
                shift
                ;;
            --log)
                output_mode="log"
                shift
                ;;
            --no-control)
                start_control="false"
                shift
                ;;
            --help)
                show_help
                exit 0
                ;;
            *)
                log_error "未知参数: $1"
                show_help
                exit 1
                ;;
        esac
    done
    
    log_info "启动参数: debug=$debug_mode, output=$output_mode, start_control=$start_control"
    
    # 启动系统
    if roslaunch task_planning execute_tasks.launch \
        debug:=$debug_mode \
        output:=$output_mode \
        start_control_nodes:=$start_control; then
        log_success "系统启动成功"
    else
        log_error "系统启动失败"
        exit 1
    fi
}

# 显示帮助信息
show_help() {
    echo "具身双臂机器人系统启动脚本"
    echo ""
    echo "用法: $0 [选项]"
    echo ""
    echo "选项:"
    echo "  --debug          启用调试模式"
    echo "  --log            输出到日志文件"
    echo "  --no-control     不启动控制节点"
    echo "  --help           显示此帮助信息"
    echo ""
    echo "示例:"
    echo "  $0                    # 正常启动"
    echo "  $0 --debug            # 调试模式启动"
    echo "  $0 --log --no-control # 日志模式，不启动控制节点"
}

# 清理函数
cleanup() {
    log_info "正在清理..."
    
    # 杀死所有相关进程
    pkill -f "execute_tasks" || true
    pkill -f "dual_arm_75_driver" || true
    
    log_success "清理完成"
}

# 信号处理
trap cleanup EXIT INT TERM

# 主函数
main() {
    log_info "🤖 具身双臂机器人系统启动脚本"
    log_info "=================================="
    
    # 执行检查
    check_ros_environment
    check_workspace
    check_dependencies
    
    # 启动系统
    start_system "$@"
}

# 运行主函数
main "$@"
