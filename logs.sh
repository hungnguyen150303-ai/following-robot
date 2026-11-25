#!/bin/bash
# View logs from running MiR Follower container

# Auto-detect docker-compose or docker compose
if command -v docker-compose &> /dev/null; then
    DOCKER_COMPOSE="docker-compose"
elif docker compose version &> /dev/null 2>&1; then
    DOCKER_COMPOSE="docker compose"
else
    echo "❌ docker-compose not found!"
    exit 1
fi

echo "🔍 MiR Follower - Log Viewer"
echo "========================================"
echo ""

# Check if container is running
if ! docker ps | grep -q "mir_follower"; then
    echo "❌ Container không chạy!"
    echo "Hãy chạy './run_mir.sh' trước"
    exit 1
fi

echo "✅ Container đang chạy"
echo ""
echo "Chọn chế độ xem logs:"
echo ""
echo "  1) Tất cả logs (realtime)"
echo "  2) Chỉ xem Gestures"
echo "  3) Chỉ xem Tracking (TARGET)"
echo "  4) Chỉ xem Navigation"
echo "  5) Chỉ xem Errors & Warnings"
echo "  6) Chỉ xem khởi động (LOADING/READY)"
echo "  7) Tùy chỉnh (nhập pattern)"
echo "  8) Attach vào container (interactive shell)"
echo ""
read -p "Chọn (1-8): " choice

case $choice in
    1)
        echo ""
        echo "📊 Xem tất cả logs (Ctrl+C để thoát)..."
        echo "========================================"
        $DOCKER_COMPOSE logs -f --tail=50
        ;;
    2)
        echo ""
        echo "✋ Xem logs Gestures..."
        echo "========================================"
        $DOCKER_COMPOSE logs -f --tail=20 | grep --color=always -E "GESTURE|gesture"
        ;;
    3)
        echo ""
        echo "🎯 Xem logs Tracking..."
        echo "========================================"
        $DOCKER_COMPOSE logs -f --tail=20 | grep --color=always -E "TARGET|Following|LOST|FOUND"
        ;;
    4)
        echo ""
        echo "🗺️  Xem logs Navigation..."
        echo "========================================"
        $DOCKER_COMPOSE logs -f --tail=20 | grep --color=always -E "NAVIGATION|GOAL|move_base"
        ;;
    5)
        echo ""
        echo "⚠️  Xem Errors & Warnings..."
        echo "========================================"
        $DOCKER_COMPOSE logs -f --tail=50 | grep --color=always -E "ERROR|WARN|FAIL|error|warning"
        ;;
    6)
        echo ""
        echo "🚀 Xem logs khởi động..."
        echo "========================================"
        $DOCKER_COMPOSE logs | grep --color=always -E "LOADING|INITIALIZING|READY|Camera|GPU|YOLO|FACE|GESTURE|ROBOT"
        ;;
    7)
        echo ""
        read -p "Nhập pattern (vd: CAMERA|GPU): " pattern
        echo ""
        echo "🔍 Lọc logs với pattern: $pattern"
        echo "========================================"
        $DOCKER_COMPOSE logs -f --tail=30 | grep --color=always -E "$pattern"
        ;;
    8)
        echo ""
        echo "🐚 Attach vào container..."
        echo "========================================"
        echo "Bạn sẽ vào shell của container. Gõ 'exit' để thoát."
        echo ""
        docker exec -it mir_follower_app bash
        ;;
    *)
        echo "❌ Lựa chọn không hợp lệ!"
        exit 1
        ;;
esac
