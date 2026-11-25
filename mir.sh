#!/bin/bash
# Quick commands for MiR Follower

# Auto-detect docker-compose or docker compose
if command -v docker-compose &> /dev/null; then
    DOCKER_COMPOSE="docker-compose"
elif docker compose version &> /dev/null 2>&1; then
    DOCKER_COMPOSE="docker compose"
else
    echo "❌ docker-compose not found!"
    exit 1
fi

# Show interactive menu
show_menu() {
    echo ""
    echo "🤖 MiR Follower - Quick Commands"
    echo "========================================"
    echo ""
    echo "  1) 🚀 Start application"
    echo "  2) 🛑 Stop application"
    echo "  3) 📊 View logs (interactive)"
    echo "  4) 🔍 Show status"
    echo "  5) 🐚 Enter container shell"
    echo "  6) 🔄 Restart application"
    echo "  7) � Rebuild (only changed files)"
    echo "  8) �🗑️  Clean & rebuild (from scratch)"
    echo "  9) ❌ Exit"
    echo ""
    read -p "Chọn (1-9): " choice
    echo ""
    
    case $choice in
        1)
            echo "🚀 Starting MiR Follower..."
            ./run_mir.sh
            ;;
        2)
            echo "🛑 Stopping MiR Follower..."
            $DOCKER_COMPOSE down
            echo "✅ Stopped"
            show_menu
            ;;
        3)
            ./logs.sh
            show_menu
            ;;
        4)
            echo "🔍 Container Status:"
            echo "========================================"
            $DOCKER_COMPOSE ps
            echo ""
            echo "📊 Resource Usage:"
            echo "========================================"
            docker stats --no-stream mir_follower_app 2>/dev/null || echo "Container không chạy"
            echo ""
            read -p "Nhấn Enter để tiếp tục..."
            show_menu
            ;;
        5)
            echo "🐚 Entering container shell..."
            docker exec -it mir_follower_app bash
            show_menu
            ;;
        6)
            echo "🔄 Restarting..."
            $DOCKER_COMPOSE restart
            echo "✅ Restarted"
            show_menu
            ;;
        7)
            echo "� Rebuilding changed files only..."
            $DOCKER_COMPOSE build
            echo "✅ Rebuild complete. Start with option 1."
            show_menu
            ;;
        8)
            echo "�🗑️  Cleaning all cache..."
            $DOCKER_COMPOSE down -v
            docker system prune -f
            echo "✅ Cleaned. Rebuilding from scratch..."
            $DOCKER_COMPOSE build --no-cache
            echo "✅ Build complete. Start with option 1."
            show_menu
            ;;
        9)
            echo "👋 Goodbye!"
            exit 0
            ;;
        *)
            echo "❌ Lựa chọn không hợp lệ!"
            show_menu
            ;;
    esac
}

# Start menu
show_menu
