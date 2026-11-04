#!/usr/bin/env python3
import json
import socket
import threading
import time
import datetime
import os
import subprocess
import pty
import codecs
import fcntl
import termios
import struct
import select
import re

try:
    import netifaces
except (ImportError, ModuleNotFoundError):
    netifaces = None
    pass
import ipaddress
import requests
import shutil

import logging
log = logging.getLogger('werkzeug')
log.setLevel(logging.ERROR)

from flask import Flask, request, jsonify, send_from_directory, Response, render_template_string, send_file

try:
    from openpilot.common.params import Params, ParamKeyType
    params = Params()
except ImportError:
    Params = None
    ParamKeyType = None
    params = None

# =========================
# 기본 설정
# =========================
UDP_PORT = 5004
HTTP_PORT = 5005
UDP_INTERVAL = 2.0
CLIENT_TIMEOUT = 5.0   # 초

PARAM_SCHEMA_PATH = "/data/openpilot/openpilot/selfdrive/kisapilot/param_schema.json"
CMD_SCHEMA_PATH = "/data/openpilot/openpilot/selfdrive/kisapilot/cmd_schema.json"

app = Flask(__name__)

client_connected = False
last_seen = 0.0
running_cmds = {}
PTY_SESSIONS = {}


def is_client_alive():
    return client_connected and (time.time() - last_seen < CLIENT_TIMEOUT)


def ffmpeg_mp4_wrap_process_builder(filename):
  command_line = [
      "ffmpeg",      "-f", "hevc",
      "-r", "20",
      "-i", filename,
      "-c", "copy",
      "-vtag", "hvc1",
      "-movflags", "faststart+frag_keyframe+default_base_moof",
      "-f", "mp4",
      "-"
  ]
  return subprocess.Popen(command_line, stdout=subprocess.PIPE)

def video_to_img(input_path, output_path):
    if os.path.exists(output_path):
        return
    try:
        command = []
        frame_to_extract = 5
        time_to_extract = "3"

        if input_path.endswith('.hevc'):
            print(f"-> Using HEVC optimized command for: {input_path}")
            command = [
                "ffmpeg",
                "-skip_frame", "nokey",
                "-i", input_path,
                "-vf", f"select=gte(n\,{frame_to_extract}),scale=640:-1",
                "-frames:v", "1",
                "-vsync", "0",
                "-an", "-sn", "-dn",
                "-q:v", "3",
                "-y", output_path
            ]
        else:
            print(f"-> Using MP4 optimized command for: {input_path}")
            command = [
                "ffmpeg",
                "-i", input_path,
                "-ss", time_to_extract,
                "-vf", "scale=640:-1",
                "-frames:v", "1",
                "-y", output_path
            ]

        subprocess.run(command, check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        print(f"Thumbnail created: {output_path}")
    except Exception as e:
        print(f"Failed to create thumbnail for {input_path}: {e}")

def update_thumbnails_in_background(path):
    try:
        current_folder = os.path.basename(path)
        
        # --- ScreenRecs
        if current_folder == 'videos':
            videos_thumbnail_dir = "/data/media/0/videos"
            if not os.path.exists(videos_thumbnail_dir):
                os.makedirs(videos_thumbnail_dir)

            for item_name in sorted(os.listdir(path)):
                item_path = os.path.join(path, item_name)
                if not os.path.isdir(item_path) and item_name.endswith('.mp4'):
                    mp4_path = item_path
                    preview_path = os.path.join(videos_thumbnail_dir, os.path.splitext(item_name)[0] + '.jpg')
                    video_to_img(mp4_path, preview_path)
            return

        # --- Drive Logs
        if current_folder == 'realdata':
            routes = {}
            for item_name in os.listdir(path):
                item_path = os.path.join(path, item_name)
                if os.path.isdir(item_path):
                    last_dash_index = item_name.rfind('--')
                    if last_dash_index > 0:
                        route_name = item_name[:last_dash_index]
                        
                        if route_name not in routes:
                            routes[route_name] = []
                        routes[route_name].append(item_name)

            for route_name, segments in routes.items():
                try:
                    segments.sort(key=lambda s: int(s.split('--')[-1]))
                except (ValueError, IndexError):
                    print(f"Warning: Could not sort segments for {route_name}, using default order.")
                if not segments:
                    continue

                first_segment_name = segments[0]
                first_segment_path = os.path.join(path, first_segment_name)
                
                fcamera_path = os.path.join(first_segment_path, "fcamera.hevc")
                preview_path = os.path.join(first_segment_path, "preview.jpg")

                if os.path.exists(fcamera_path):
                    if not os.path.exists(preview_path) or os.path.getmtime(fcamera_path) > os.path.getmtime(preview_path):
                        print(f"Generating thumbnail for the first segment: {first_segment_name}")
                        video_to_img(fcamera_path, preview_path)

    except Exception as e:
        print(f"[BG Thread] Error during thumbnail generation: {e}")

def get_item_details(item_path):
    is_dir = os.path.isdir(item_path)
    name = os.path.basename(item_path)
    try:
        size = os.path.getsize(item_path)
        modified_time = os.path.getmtime(item_path)
        modified_date = datetime.datetime.fromtimestamp(modified_time, datetime.timezone(datetime.timedelta(hours=9))).strftime('%Y-%m-%d %H:%M:%S')
    except Exception:
        size = -1
        modified_time = None
        modified_date = "N/A"
    return {"name": name, "path": item_path, "is_dir": is_dir, "size": size, "modified": modified_date, "modified_unix": modified_time, "modified_timezone": "Asia/Seoul"}

@app.route('/files', methods=['GET'])
def list_files():
    path = request.args.get('path', '/data/openpilot')
    
    if not os.path.isdir(path):
        return jsonify({"error": "Path is not a valid directory"}), 404

    items = []
    try:
        for item_name in sorted(os.listdir(path)):
            try:
                items.append(get_item_details(os.path.join(path, item_name)))
            except Exception:
                pass
        return jsonify({"path": path, "items": items})
    except Exception as e:
        return jsonify({"error": str(e)}), 500

@app.route("/file/explorer_delete", methods=["POST"])
def file_delete():
    data = request.get_json(silent=True) or {}
    path = data.get("path")

    if not path or not os.path.exists(path):
        return jsonify({"error": "File or directory not found"}), 404

    try:
        if os.path.isdir(path):
            shutil.rmtree(path)
        else:
            os.remove(path)
        print(f"Deleted: {path}")
        return jsonify({"status": "deleted", "path": path})
    except Exception as e:
        print(f"Error deleting {path}: {e}")
        return jsonify({"error": str(e)}), 500

@app.route("/file/explorer_rename", methods=["POST"])
def file_rename():
    data = request.get_json(silent=True) or {}
    old_path = data.get("old_path")
    new_name = data.get("new_name")

    if not all([old_path, new_name]) or not os.path.exists(old_path):
        return jsonify({"error": "Invalid arguments or file not found"}), 400

    try:
        directory = os.path.dirname(old_path)
        new_path = os.path.join(directory, new_name)

        os.rename(old_path, new_path)
        print(f"Renamed: {old_path} -> {new_path}")
        return jsonify({"status": "renamed", "new_path": new_path})
    except Exception as e:
        print(f"Error renaming {old_path}: {e}")
        return jsonify({"error": str(e)}), 500

@app.route("/file/explorer_download", methods=["GET"])
def file_download():
    file_path = request.args.get("path")

    if not file_path or not os.path.exists(file_path) or not os.path.isfile(file_path):
        return "File not found or is not a file", 404

    try:
        return send_file(file_path, as_attachment=True)
    except Exception as e:
        print(f"Error sending file {file_path}: {e}")
        return str(e), 500


@app.route("/file/explorer_copy", methods=["POST"])
def file_copy():
    data = request.get_json(silent=True) or {}
    source_path = data.get("source_path")
    destination_folder = data.get("destination_path")

    if not all([source_path, destination_folder]):
        return jsonify({"error": "Source or destination path is missing"}), 400

    try:
        source_name = os.path.basename(source_path)
        destination_path = os.path.join(destination_folder, source_name)

        counter = 1
        base_name, extension = os.path.splitext(source_name)
        
        while os.path.exists(destination_path):
            if counter == 1:
                new_name = f"{base_name} {extension}_copy"
            else:
                new_name = f"{base_name} {extension}_copy_{counter}"
            
            destination_path = os.path.join(destination_folder, new_name)
            counter += 1
        if os.path.isdir(source_path):
            shutil.copytree(source_path, destination_path)
        else:
            shutil.copy2(source_path, destination_path)
            
        print(f"Copied: {source_path} -> {destination_path}")
        return jsonify({"status": "copied"})
    except Exception as e:
        print(f"Error copying {source_path}: {e}")
        return jsonify({"error": str(e)}), 500

# Bookmark exports preserve original segment paths for replay analysis.
def bookmark_recording_context():
    from openpilot.selfdrive.kisapilot.bookmark_export import recording_routes
    # An unavailable recording state must retain the normal lock protection.
    if params is None:
        return True, None
    onroad = not params.get_bool("IsOffroad")
    route = params.get("CurrentRoute") or b''
    if isinstance(route, bytes):
        route = route.decode('utf-8', errors='replace')
    return onroad, recording_routes('/data/media/0/realdata', onroad, route)



@app.route('/bookmarks', methods=['GET'])
def list_saved_bookmarks():
    from openpilot.selfdrive.kisapilot.bookmark_export import list_bookmarks
    try:
        _, recording = bookmark_recording_context()
        return jsonify(list_bookmarks('/data/media/0/realdata', recording=recording))
    except FileNotFoundError as e:
        return jsonify({"error": str(e)}), 404
    except Exception as e:
        return jsonify({"error": str(e)}), 500


@app.route('/bookmarks/auto', methods=['GET'])
def automatic_bookmark_status():
    from openpilot.selfdrive.kisapilot.bookmark_export import auto_bookmarks
    try:
        onroad, recording = bookmark_recording_context()
        return jsonify(auto_bookmarks('/data/media/0/realdata', onroad, recording=recording))
    except FileNotFoundError as e:
        return jsonify({"error": str(e)}), 404
    except Exception as e:
        return jsonify({"error": str(e)}), 500


@app.route('/bookmarks/auto/status', methods=['POST'])
def automatic_bookmark_transfer_status():
    from openpilot.selfdrive.kisapilot.bookmark_status import sanitize_report
    if request.content_length is None or request.content_length > 2048:
        return jsonify({"error": "Invalid status size"}), 400
    try:
        report = sanitize_report(request.get_json(silent=True), time.monotonic())
        Params('/dev/shm/params').put('KisaBookmarkTransferStatus', json.dumps(report))
        return jsonify({"ok": True})
    except (ValueError, TypeError) as e:
        return jsonify({"error": str(e)}), 400
    except Exception as e:
        return jsonify({"error": str(e)}), 500


@app.route('/bookmarks/export', methods=['POST'])
def prepare_bookmark_export():
    from openpilot.selfdrive.kisapilot.bookmark_export import export_bookmarks
    data = request.get_json(silent=True)
    if not isinstance(data, dict):
        return jsonify({"error": "JSON 요청이 필요합니다."}), 400
    try:
        _, recording = bookmark_recording_context()
        return jsonify(export_bookmarks('/data/media/0/realdata', data.get('ids'), data.get('adjacent', 1),
                                       recording=recording, archive=data.get('archive', True)))
    except (ValueError, FileNotFoundError) as e:
        return jsonify({"error": str(e)}), 409
    except Exception as e:
        return jsonify({"error": str(e)}), 500


@app.route('/realdata_files', methods=['GET'])
def list_realdata_files_recursively():
    path = '/data/media/0/realdata'

    if not os.path.isdir(path):
        return jsonify({"error": "Realdata path not found"}), 404

    thread = threading.Thread(target=update_thumbnails_in_background, args=(path,))
    thread.start()

    items = []
    try:
        for dirpath, dirnames, filenames in os.walk(path):
            if dirpath == path:
                for dirname in dirnames:
                    try: items.append(get_item_details(os.path.join(dirpath, dirname)))
                    except Exception: pass
            for filename in filenames:
                try: items.append(get_item_details(os.path.join(dirpath, filename)))
                except Exception: pass
        
        return jsonify({"path": path, "items": items})
    except Exception as e:
        return jsonify({"error": str(e)}), 500

@app.route('/videos_files', methods=['GET'])
def list_videos_files_filtered():
    path = '/data/media/0/videos'

    if not os.path.isdir(path):
        return jsonify({"error": "Videos path not found"}), 404

    thread = threading.Thread(target=update_thumbnails_in_background, args=(path,))
    thread.start()

    items = []
    try:
        for item_name in sorted(os.listdir(path)):
            if item_name.endswith('.mp4'):
                try:
                    items.append(get_item_details(os.path.join(path, item_name)))
                except Exception:
                    pass
        
        return jsonify({"path": path, "items": items})
    except Exception as e:
        return jsonify({"error": str(e)}), 500

@app.route('/file/image', methods=['GET'])
def get_image_content():
    file_path = request.args.get('path', '')

    if not file_path:
        return Response("Path parameter is missing", status=400)

    try:
        if os.path.exists(file_path) and os.path.isfile(file_path) and file_path.lower().endswith(('.png', '.jpg', '.jpeg', '.gif', '.bmp')):
            with open(file_path, 'rb') as f:
                image_data = f.read()
            import io
            return send_file(io.BytesIO(image_data),
                mimetype='image/jpeg'
            )
        else:
            return Response("Image not found or not a valid image file", status=404)
    except Exception as e:
        print(f"Error reading image file {file_path}: {e}")
        return Response(f"Error: {str(e)}", status=500)

@app.route('/playlist/<path:segment_path>')
def create_playlist_player(segment_path):
    try:
        route_segment_full, video_filename = os.path.split(segment_path)
        current_segment_name = os.path.basename(route_segment_full)
        route_name = current_segment_name.rsplit('--', 1)[0]
        realdata_path = '/data/media/0/realdata'
        all_segments = sorted(
            [d for d in os.listdir(realdata_path) if os.path.isdir(os.path.join(realdata_path, d)) and d.startswith(route_name)],
            key=lambda s: int(s.split('--')[-1])
        )

        playlist_details = []
        for segment in all_segments:
            video_file_path = os.path.join(realdata_path, segment, video_filename)
            video_url = f"/media/realdata/{segment}/{video_filename}"
            
            modified_time = "N/A"
            try:
                mtime = os.path.getmtime(video_file_path)
                modified_time = datetime.datetime.fromtimestamp(mtime).strftime('%Y-%m-%d %H:%M:%S')
            except FileNotFoundError:
                pass

            playlist_details.append({
                "url": video_url,
                "name": f"{segment}/{video_filename}",
                "time": modified_time
            })

        current_video_full_url = f"/media/{segment_path}"
        try:
            start_index = [item['url'] for item in playlist_details].index(current_video_full_url)
        except ValueError:
            start_index = 0

        return render_template_string('''
        <!DOCTYPE html>
        <html>
        <head>
            <title>KisaPilot Video Player</title>
            <meta name="viewport" content="width=device-width, initial-scale=1.0, user-scalable=no">

            <style>
                html, body {
                    margin: 0;
                    padding: 0;
                    width: 100%;
                    height: 100%;
                    background-color: black;
                    overflow: hidden;
                    color: white;
                    font-family: sans-serif;
                }

                .video-container {
                    position: relative;
                    width: 100%;
                    height: 100%;
                    display: flex;
                    justify-content: center;
                    align-items: center;
                }

                video {
                    max-width: 100%;
                    max-height: calc(100% - 100px);
                    object-fit: contain;
                }

                .info-overlay {
                    position: absolute;
                    top: 10px;
                    left: 10px;
                    background-color: rgba(0, 0, 0, 0.6);
                    padding: 8px 12px;
                    border-radius: 8px;
                    font-family: monospace;
                    font-size: 14px;
                    opacity: 0;
                    transition: opacity 0.3s;
                    pointer-events: none;
                }

                .controls {
                    position: fixed;
                    bottom: calc(20px + env(safe-area-inset-bottom));
                    left: 50%;
                    transform: translateX(-50%);
                    display: flex;
                    gap: 24px;
                    background-color: rgba(0, 0, 0, 0.6);
                    padding: 14px 28px;
                    border-radius: 30px;
                    opacity: 0;
                    transition: opacity 0.3s;
                    z-index: 10;
                }

                .controls button {
                    font-size: 22px;
                    padding: 10px 18px;
                    min-width: 60px;
                    border-radius: 12px;
                    border: none;
                    background-color: #222;
                    color: white;
                }

                .controls button:active {
                    background-color: #444;
                }

                .video-container:hover .info-overlay,
                .video-container:hover .controls {
                    opacity: 1;
                }
            </style>
        </head>

        <body>
        <div class="video-container">

            <video id="player" controls autoplay muted playsinline></video>

            <div id="infoOverlay" class="info-overlay"></div>

            <div class="controls">
                <button id="prevBtn">«</button>
                <button id="nextBtn">»</button>
            </div>

        </div>

        <script>
            const videoPlayer = document.getElementById('player');
            const infoOverlay = document.getElementById('infoOverlay');
            const prevBtn = document.getElementById('prevBtn');
            const nextBtn = document.getElementById('nextBtn');

            const playlist = {{ playlist_details | tojson | safe }};
            let currentIndex = {{ start_index }};

            function updateInfo() {
                if (!playlist[currentIndex]) return;
                const totalVideos = playlist.length;
                infoOverlay.textContent = `[${currentIndex + 1}/${totalVideos}] ${playlist[currentIndex].time}`;
                document.title = `[${currentIndex + 1}/${totalVideos}] ${playlist[currentIndex].time}`;
            }

            function playVideo(index) {
                if (index < 0 || index >= playlist.length) return;
                currentIndex = index;
                videoPlayer.src = playlist[currentIndex].url;
                videoPlayer.load();
                videoPlayer.play();
                updateInfo();
            }

            prevBtn.onclick = () => playVideo((currentIndex - 1 + playlist.length) % playlist.length);
            nextBtn.onclick = () => playVideo((currentIndex + 1) % playlist.length);
            videoPlayer.onended = () => nextBtn.click();

            playVideo(currentIndex);
        </script>
        </body>
        </html>

        ''', playlist_details=playlist_details, start_index=start_index)

    except Exception as e:
        return f"Error creating playlist: {str(e)}", 500

@app.route('/file/delete', methods=['POST'])
def delete_files():
    data = request.get_json()
    if not data or 'paths' not in data:
        return jsonify({"error": "Paths parameter is missing in request body"}), 400

    paths_to_delete = data['paths']
    if not isinstance(paths_to_delete, list):
        return jsonify({"error": "Paths parameter must be a list"}), 400

    allowed_dirs = ['/data/media/0/videos', '/data/media/0/realdata']
    deleted_count = 0
    errors = []

    for item_path in paths_to_delete:
        try:
            is_allowed = any(os.path.commonpath([allowed_dir, os.path.abspath(item_path)]) == allowed_dir for allowed_dir in allowed_dirs)
            if not is_allowed:
                errors.append(f"Deletion not allowed for: {item_path}")
                continue

            if os.path.isdir(item_path):
                shutil.rmtree(item_path)
                print(f"Directory deleted: {item_path}")
                deleted_count += 1
            elif os.path.exists(item_path):
                os.remove(item_path)
                thumbnail_path = os.path.splitext(item_path)[0] + '.jpg'
                if os.path.exists(thumbnail_path):
                    os.remove(thumbnail_path)
                print(f"File deleted: {item_path}")
                deleted_count += 1
            else:
                errors.append(f"Not found: {item_path}")

        except Exception as e:
            errors.append(f"Error deleting {item_path}: {str(e)}")
            print(f"Error deleting {item_path}: {e}")

    if not errors:
        return jsonify({"success": True, "message": f"{deleted_count} items deleted."})
    else:
        return jsonify({"success": False, "message": f"{deleted_count} items deleted, but some errors occurred.", "errors": errors}), 207


@app.route('/file/exists', methods=['GET'])
def file_exists():
    path = request.args.get('path')
    if not path:
        return jsonify({"error": "Path parameter is missing"}), 400

    if os.path.exists(path):
        return jsonify({"exists": True, "path": path})
    else:
        return jsonify({"exists": False, "path": path})


@app.route('/media/<path:filename>')
def serve_media(filename):
    file_path = os.path.join('/data/media/0', filename)

    if not os.path.exists(file_path):
        return "File not found", 404

    if file_path.lower().endswith('.hevc'):
        process = ffmpeg_mp4_wrap_process_builder(file_path)
        
        def generate():
            with process.stdout as pipe:
                while True:
                    chunk = pipe.read(4096)
                    if not chunk:
                        break
                    yield chunk
        
        return Response(generate(), mimetype='video/mp4')
    
    else:
        return send_from_directory('/data/media/0', filename)


@app.route('/file/size', methods=['GET'])
def get_file_size():
    file_path = request.args.get("path")
    if not file_path:
        return jsonify({"error": "missing file path"}), 400

    try:
        if os.path.exists(file_path):
            size = os.path.getsize(file_path)
            return jsonify({"path": file_path, "size": size})
        else:
            return jsonify({"path": file_path, "size": -1})
    except Exception as e:
        return jsonify({"error": str(e)}), 500

@app.route('/file/remote_size', methods=['GET'])
def get_remote_file_size():
    url = request.args.get("url")
    if not url:
        return jsonify({"error": "missing url"}), 400

    try:
        response = requests.head(url, timeout=5, allow_redirects=True)
        response.raise_for_status()
        
        size = int(response.headers.get('Content-Length', -1))
        
        return jsonify({"url": url, "size": size})
    except Exception as e:
        return jsonify({"error": str(e), "size": -1}), 500


# =========================
# Param get / set
# =========================
@app.route("/param_schema.json", methods=["GET"])
def get_param_schema():
    try:
        if not os.path.exists(PARAM_SCHEMA_PATH):
            return jsonify({"error": "Schema file not found"}), 404

        with open(PARAM_SCHEMA_PATH, "r") as f:
            schema_list = json.load(f)

        for item in schema_list:
            key = item.get("param")
            if key:
                current_val, val_type = get_param_value(key)

                if current_val is None:
                    default_val = item.get("default", "")
                    item["value"] = str(default_val)
                else:
                    if isinstance(current_val, bytes):
                         try:
                             item["value"] = current_val.decode("utf-8").strip()
                         except:
                             item["value"] = str(current_val)
                    else:
                        item["value"] = str(current_val)
                
                item["value_type"] = val_type

        return jsonify(schema_list)

    except Exception as e:
        return jsonify({"error": str(e)}), 500

def get_param_value(key: str):
    param_type = params.get_type(key)

    if param_type == ParamKeyType.BOOL:
        return params.get_bool(key), "BOOL"

    val = params.get(key)
    if val is None:
        return None, "STRING"

    try:
        if param_type == ParamKeyType.INT:
            return int(val), "INT"
        elif param_type == ParamKeyType.FLOAT:
            return float(val), "FLOAT"
        else:
            return val, "STRING"
    except Exception:
        return val, "STRING"


def set_param_value(key: str, val):
    param_type = params.get_type(key)

    if param_type == ParamKeyType.BOOL:
        if isinstance(val, str):
            val_bool = val.lower() in ("1", "true", "yes", "on")
        else:
            val_bool = bool(val)
        params.put_bool(key, val_bool)
    else:
        if param_type == ParamKeyType.INT:
            params.put(key, int(val))
        elif param_type == ParamKeyType.FLOAT:
            params.put(key, float(val))
        else:
            params.put(key, str(val))


@app.route("/param/get_all", methods=["GET"])
def param_get_all():
    try:
        with open(PARAM_SCHEMA_PATH, "r") as f:
            schema = json.load(f)
            
        all_values = {}
        for item in schema:
            key = item["param"]
            val, _ = get_param_value(key)
            all_values[key] = val
            
        return jsonify(all_values)
    except Exception as e:
        return jsonify({"error": str(e)}), 500

@app.route("/log/tmux", methods=["GET"])
def get_tmux_log():
    try:
        # tmux 세션의 화면을 캡처하는 명령어
        # -pt 0 : 타겟 세션 '0'의 패널 내용을 출력(p)
        # -S -100 : 최근 100줄만 가져오기 (너무 많으면 느려질 수 있음)
        # -e : ANSI 색상 코드 포함 (앱에서 색상 파싱을 하므로 포함해서 보냄)

        cmd = ["tmux", "capture-pane", "-pt", "comma", "-S", "-500", "-e", "-J"]

        
        # 명령어 실행
        # errors='ignore' 또는 'replace'를 써서 디코딩 에러 방지
        output = subprocess.check_output(cmd, encoding='utf-8', errors='replace')
        
        return jsonify({"log": output})

    except subprocess.CalledProcessError as e:
        # tmux가 실행 중이지 않거나 세션이 없을 때
        return jsonify({"log": f"Error: tmux session not found or command failed.\n{str(e)}"}), 500
        
    except Exception as e:
        return jsonify({"error": str(e)}), 500


# =========================
# HTTP API
# =========================
# Test requests are short leases consumed by card, the sole sendcan publisher.
# No parking steering packets or ECU-disable requests are issued here.
_vehicle_test_lock = threading.RLock()
_vehicle_test_sm = None
_vehicle_test_session = None
_vehicle_test_cooldown = 0.0


def vehicle_test_preflight():
    global _vehicle_test_sm
    from opendbc.car.hyundai.blinker_test import conditions, read_json, STATUS, SAFETY_FLAG, TBT_SAFETY_FLAG
    result = {"fresh": False, "supported": False, "conditions_met": False, "reason": "vehicle_data_missing"}
    try:
        from openpilot.cereal import messaging
        from opendbc.car.structs import CarParams
        from opendbc.car.hyundai.values import CAR, HyundaiFlags
        with _vehicle_test_lock:
            if _vehicle_test_sm is None:
                _vehicle_test_sm = messaging.SubMaster(["carState", "carControl", "selfdriveState", "pandaStates"])
            sm = _vehicle_test_sm
            sm.update(0)
            now = time.monotonic()
            result["fresh"] = all(sm.valid[name] and 0 <= now - sm.recv_time[name] < 0.5
                                  for name in ("carState", "carControl", "selfdriveState"))
            panda_fresh = sm.valid["pandaStates"] and 0 <= now - sm.recv_time["pandaStates"] < 1.5
            moving_flags = SAFETY_FLAG | TBT_SAFETY_FLAG
            moving_configured = False
            raw = params.get("CarParams") if params is not None else None
            if raw:
                with CarParams.from_bytes(raw) as cp:
                    result["supported"] = (cp.carFingerprint == CAR.HYUNDAI_IONIQ_5_PE and
                        not (cp.flags & HyundaiFlags.ENABLE_BLINKERS) and
                        any(int(c.safetyParam) & SAFETY_FLAG for c in cp.safetyConfigs))
                    moving_configured = any((int(c.safetyParam) & moving_flags) == moving_flags for c in cp.safetyConfigs)
            panda_moving_configured = panda_fresh and any(
                (int(p.safetyParam) & moving_flags) == moving_flags for p in sm["pandaStates"])
            result["moving_configured_in_car_params"] = moving_configured
            result["moving_configured_in_panda"] = bool(panda_moving_configured)
            result["moving_test_enabled"] = bool(result["supported"] and moving_configured and panda_moving_configured)
            if result["fresh"]:
                cs, cc, sd = sm["carState"], sm["carControl"], sm["selfdriveState"]
                result.update(gear=str(cs.gearShifter), stationary=bool(cs.standstill),
                    brake_pressed=bool(cs.brakePressed), gas_released=not cs.gasPressed,
                    automatic_control_off=not (cc.enabled or cc.latActive or cc.longActive or sd.enabled),
                    left_observed=bool(cs.leftBlinker), right_observed=bool(cs.rightBlinker))
                steering_angle = float(cs.steeringAngleDeg)
                if float("-inf") < steering_angle < float("inf"):
                    result["steering_angle_deg"] = steering_angle
                reason = conditions(cs, allow_moving=result["moving_test_enabled"])
                if not cs.standstill and not result["moving_test_enabled"]:
                    reason = "moving_not_configured" if not moving_configured else "moving_panda_not_ready"
                if not result["supported"]:
                    reason = "unsupported_car_or_configuration"
                result["reason"] = reason
                result["conditions_met"] = reason == "ok"
            status = read_json(STATUS)
            result["sender"] = status if 0 <= now - float(status.get("time", 0)) < .5 else {}
            if result["sender"].get("reason") in ("unsupported_car_or_bus", "stock_spas_active_or_unknown"):
                result["conditions_met"] = False
                result["reason"] = result["sender"]["reason"]
            result["panda_tx_blocked"] = sum(int(p.safetyTxBlocked) for p in sm["pandaStates"])
            result["panda_test_flag"] = bool(panda_fresh and
                any(int(p.safetyParam) & SAFETY_FLAG for p in sm["pandaStates"]))
    except Exception:
        result["fresh"] = False
        result["conditions_met"] = False
        result["reason"] = "vehicle_data_unavailable"
    return result


def _end_blinker_test(now):
    global _vehicle_test_session, _vehicle_test_cooldown
    from opendbc.car.hyundai.blinker_test import write_json, REQUEST
    if _vehicle_test_session:
        try:
            write_json(REQUEST, dict(_vehicle_test_session, action="stop", expires=now))
        except OSError:
            pass  # The unrenewed lease still expires independently in card.
        print("BLINKER_TEST end", flush=True)
    _vehicle_test_session = None
    _vehicle_test_cooldown = now + 1.1


@app.route("/tests/capabilities", methods=["GET"])
def vehicle_test_capabilities():
    preflight = vehicle_test_preflight()
    preflight.get("sender", {}).pop("id", None)
    preflight.get("sender", {}).pop("voice_id", None)
    cooldown = max(0.0, _vehicle_test_cooldown - time.monotonic())
    return jsonify({"protocol": 2, "preflight": preflight, "cooldown_seconds": cooldown, "tests": [{
        "id": "blinkers", "supported": preflight["supported"], "max_seconds": 3,
        "transmission_enabled": preflight["conditions_met"] and preflight.get("panda_test_flag", False) and
                                bool(preflight.get("sender")) and cooldown <= 0
    }]})


@app.route("/tests/steering/preview", methods=["POST"])
def vehicle_test_steering_preview():
    payload = request.get_json(silent=True)
    target = payload.get("target_angle_deg") if isinstance(payload, dict) else None
    if type(target) is not int or not -30 <= target <= 30:
        return jsonify({"status": "invalid_target_angle", "transmitted": False}), 400
    from opendbc.can import CANPacker

    # Build a diagnostic sample only. No sendcan publication or steering session.
    packer = CANPacker("hyundai_canfd_generated")
    address, data, _ = packer.make_can_msg("SPAS1", 0, {
        "NEW_SIGNAL_1": target,
        "NEW_SIGNAL_2": 3,
    })
    if address != 0x165 or len(data) != 24:
        return jsonify({"status": "preview_pack_failed", "transmitted": False}), 503
    preflight = vehicle_test_preflight()
    angle = preflight.get("steering_angle_deg") if preflight["fresh"] else None
    return jsonify({"status": "preview", "target_angle_deg": target,
                    "steering_angle_deg": angle, "transmitted": False,
                    "can_preview": {"address": address, "length": len(data), "data_hex": data.hex()}})


@app.route("/tests/blinkers", methods=["POST"])
def vehicle_test_blinkers():
    global _vehicle_test_session
    import secrets
    from opendbc.car.hyundai.blinker_test import write_json, REQUEST, LEASE_SECONDS, MAX_SECONDS
    payload = request.get_json(silent=True)
    if not isinstance(payload, dict) or payload.get("action") not in ("left", "right", "stop", "renew"):
        return jsonify({"status": "invalid_request"}), 400
    with _vehicle_test_lock:
        now = time.monotonic()
        session = _vehicle_test_session
        action = payload["action"]
        owner = request.remote_addr
        token = payload.get("token")
        if session and now >= session["expires"]:
            _end_blinker_test(now)
            session = None
        authenticated = bool(session and isinstance(token, str) and token.isascii() and len(token) < 128 and
                             secrets.compare_digest(token, session["id"]) and session["owner"] == owner)
        if action == "stop":
            if authenticated:
                _end_blinker_test(now)
            return jsonify({"status": "stopped" if authenticated else "no_matching_session"})
        preflight = vehicle_test_preflight()
        sender = preflight.get("sender", {})
        if not (preflight["conditions_met"] and preflight.get("panda_test_flag") and sender):
            if authenticated:
                _end_blinker_test(now)
            preflight.get("sender", {}).pop("id", None)
            preflight.get("sender", {}).pop("voice_id", None)
            return jsonify({"status": "blocked", "preflight": preflight}), 409
        if action == "renew":
            if not authenticated:
                return jsonify({"status": "session_mismatch"}), 409
            if (now >= session["expires"] or now >= session["deadline"] or
                (sender.get("id") == session["id"] and sender.get("reason") != "ok")):
                _end_blinker_test(now)
                return jsonify({"status": "expired_or_cancelled"}), 409
            session["expires"] = min(now + LEASE_SECONDS, session["deadline"])
        else:
            if session and now < session["expires"]:
                return jsonify({"status": "busy"}), 409
            if now < _vehicle_test_cooldown or (sender.get("command") not in (None, 0)):
                return jsonify({"status": "cooldown"}), 409
            # Both lamps must be off before beginning; do not override a stalk/hazards.
            if preflight.get("left_observed") or preflight.get("right_observed"):
                return jsonify({"status": "existing_blinker_or_hazard"}), 409
            session = {"id": secrets.token_urlsafe(24), "owner": owner, "start": now,
                       "expires": now + LEASE_SECONDS, "deadline": now + MAX_SECONDS,
                       "action": action}
            _vehicle_test_session = session
            print(f"BLINKER_TEST start direction={action} max_seconds={MAX_SECONDS}", flush=True)
        try:
            write_json(REQUEST, session)
        except OSError:
            _vehicle_test_session = None
            return jsonify({"status": "request_write_failed"}), 503
        return jsonify({"status": "requested", "token": session["id"],
                        "seconds_left": max(0, session["deadline"] - now),
                        "note": "CAN request is not proof of lamp activation."})


@app.route("/health", methods=["GET"])
def health():
    global client_connected, last_seen
    client_connected = True
    last_seen = time.time()

    return jsonify({
        "status": "ok",
        "device": "kisapilot",
        "port": HTTP_PORT
    })


@app.route("/ping", methods=["GET"])
def ping():
    global client_connected, last_seen
    client_connected = True
    last_seen = time.time()
    return "ok"


@app.route("/param/schema", methods=["GET"])
def param_schema():
    with open(PARAM_SCHEMA_PATH, "r") as f:
        return jsonify(json.load(f))


@app.route("/param/get", methods=["GET"])
def param_get():
    key = request.args.get("name")
    if not key:
        return jsonify({"error": "missing param name"}), 400

    value, value_type = get_param_value(key)

    return jsonify({
        "param": key,
        "value": value,
        "value_type": value_type
    })


@app.route("/param/set", methods=["POST"])
def param_set():
    data = request.json
    key = data.get("param")
    val = data.get("value")

    if key is None:
        return jsonify({"error": "missing param"}), 400

    if key == "KisaRebootRequest":
        return jsonify({"error": "Reserved reboot coordination parameter"}), 400
    if key == "DoReboot" and (val is True or val == 1 or str(val).lower() in ("1", "true")):
        return jsonify(start_agent_command("reboot"))
    set_param_value(key, val)

    if key == "UpdaterTargetBranch":
        try:
            # -SIGHUP : 즉시 다운로드 시작 (Fetch)
            os.system("pkill -SIGHUP -f system.updated.updated")
            print(f"Executed os.system(pkill -SIGHUP) for branch: {val}")
        except Exception as e:
            print(f"Failed to execute pkill: {e}")

    return jsonify({"status": "ok"})

@app.route("/param/remove", methods=["POST"])
def param_remove():
    data = request.json
    key = data.get("param")

    if key is None:
        return jsonify({"error": "missing param to remove"}), 400

    params.remove(str(key))

    return jsonify({"status": "ok", "message": f"param '{key}' removed"})

@app.route("/cmd/list", methods=["GET"])
def cmd_list():
    try:
        with open(CMD_SCHEMA_PATH, "r") as f:
            schema = json.load(f)

        cmds = []
        for c in schema:
            cmds.append({
                "title": c.get("title"),
                "cmd": c.get("cmd"),
                "description": c.get("description", ""),
                "confirm": c.get("confirm", False),
                "onroad": c.get("onroad", True)
            })

        return jsonify(cmds)

    except FileNotFoundError:
        return jsonify({"error": "cmd_schema.json not found"}), 500
    except Exception as e:
        return jsonify({"error": str(e)}), 500


import uuid
current_working_directory = "/data/openpilot"
@app.route("/reboot/capabilities", methods=["GET"])
def reboot_capabilities():
    try:
        if params is None:
            raise RuntimeError("Params unavailable")
        params.get("KisaRebootRequest")
        return jsonify({"disengageBeforeReboot": True, "version": 1})
    except Exception as error:
        return jsonify({"disengageBeforeReboot": False, "error": str(error)}), 503


REBOOT_COMMANDS = {
    "reboot": ([], False),
    "sudo_reboot": ([], True),
    "git_pull_reboot": (["bash", "-c", "git pull && touch /data/ks && rm -f /data/openpilot/prebuilt"], False),
    "git_pull_reboot_force": (["bash", "-c", "git pull --force && touch /data/ks && rm -f /data/openpilot/prebuilt"], True),
    "git_reset_reboot": (["bash", "-c", "git fetch && git reset --hard $(git rev-parse --symbolic-full-name @{u}) && touch /data/ks && rm -f /data/openpilot/prebuilt"], False),
    "git_reset_reboot_force": (["bash", "-c", "git fetch && git reset --hard $(git rev-parse --symbolic-full-name @{u}) && touch /data/ks && rm -f /data/openpilot/prebuilt"], True),
}


def start_agent_command(cmd_id, param=None):
    if not isinstance(cmd_id, str):
        raise ValueError("Missing command")
    reboot = cmd_id in REBOOT_COMMANDS or cmd_id == "git_restore_reboot"
    force = False
    if cmd_id == "git_restore_reboot":
        if not isinstance(param, str) or not re.fullmatch(r"[0-9a-fA-F]{7,40}", param):
            raise ValueError("Invalid restore commit")
        command = ["bash", "-c", "git reset --hard "+param+" && touch /data/ks && rm -f /data/openpilot/prebuilt"]
        force = True
    elif cmd_id in REBOOT_COMMANDS:
        command, force = REBOOT_COMMANDS[cmd_id]
    elif cmd_id == "git_log_export":
        command = ["bash", "-c", "git log --date=human --pretty=format:'%h, %ad : %s' -n 30 > /data/params/d/GitCommits"]
    elif cmd_id == "pre_install_cleanup":
        command = ["bash", "-c", "touch /data/ks && rm -f /data/openpilot/prebuilt"]
    else:
        with open(CMD_SCHEMA_PATH) as stream:
            entry = next((c for c in json.load(stream) if c["cmd"] == cmd_id), None)
        if not entry or not entry.get("exec"):
            raise ValueError("Unknown command")
        command = entry["exec"]
        if re.search(r"\b(reboot|shutdown|poweroff)\b|DoReboot|KisaRebootRequest", " ".join(command), re.I):
            raise ValueError("Use a guarded reboot command")
    cmd_uuid = str(uuid.uuid4())
    running_cmds[cmd_uuid] = {"output": "", "done": False}
    def write(message):
        running_cmds[cmd_uuid]["output"] += message + "\n"
    def execute():
        if command:
            proc = subprocess.Popen(command, cwd=current_working_directory, stdout=subprocess.PIPE,
                                    stderr=subprocess.STDOUT, text=True)
            for line in proc.stdout:
                running_cmds[cmd_uuid]["output"] += line
            if proc.wait() != 0:
                raise RuntimeError("Command failed; reboot cancelled")
    def runner():
        try:
            if reboot:
                from openpilot.selfdrive.kisapilot.reboot_guard import RebootGuard
                with RebootGuard(params, write) as guard:
                    execute()
                    guard.reboot(force)
            else:
                execute()
        except Exception as error:
            write(f"Operation stopped: {error}")
        finally:
            running_cmds[cmd_uuid]["done"] = True
    threading.Thread(target=runner, daemon=True).start()
    return {"status": "started", "id": cmd_uuid}


@app.route("/cmd/run", methods=["POST"])
def run_cmd():
    data = request.get_json(silent=True) or {}
    try:
        return jsonify(start_agent_command(data.get("id"), data.get("param")))
    except ValueError as error:
        return jsonify({"error": str(error)}), 400


@app.route("/cmd/exec_raw", methods=["POST"])
def exec_raw_cmd():
    try:
        data = request.json
        cmd_str = data.get("cmd")

        if not cmd_str:
            return jsonify({"error": "missing cmd"}), 400

        if re.search(r"\b(reboot|shutdown|poweroff)\b|DoReboot|KisaRebootRequest", cmd_str, re.I):
            return jsonify({"error": "Use /cmd/run guarded reboot commands"}), 400
        print(f"Executing RAW command: {cmd_str}")

        def runner():
            subprocess.run(cmd_str, shell=True)

        threading.Thread(target=runner, daemon=True).start()

        return jsonify({"status": "started", "cmd": cmd_str})

    except Exception as e:
        return jsonify({"error": str(e)}), 500


@app.route("/cmd/output", methods=["GET"])
def cmd_output():
    cmd_id = request.args.get("id")
    if not cmd_id or cmd_id not in running_cmds:
        return jsonify({"error": "invalid id"}), 404

    return jsonify(running_cmds[cmd_id])


@app.route('/file/content', methods=['GET'])
def get_file_content():
    path = request.args.get('path')

    if not path or not os.path.exists(path) or os.path.isdir(path):
        return "File not found or is a directory.", 404

    try:
        with open(path, 'r', errors='ignore') as f:
            content = f.read()
            return content
    except Exception as e:
        return f"Error reading file: {str(e)}", 500


@app.route('/execute_fingerprint', methods=['GET'])
def execute_fingerprint():
    realdata_path = '/data/media/0/realdata'
    
    if not os.path.exists(realdata_path) or not os.path.isdir(realdata_path):
        return jsonify({"error": "Drive log directory (/data/media/0/realdata) not found."}), 404

    all_segments = [d for d in os.listdir(realdata_path) if os.path.isdir(os.path.join(realdata_path, d))]
    if not all_segments:
        return jsonify({"error": "No drive logs found. Please enable the drive log toggle and drive for about 10 minutes."}), 404
    
    latest_segment_folder = sorted(all_segments)[-1]
    qlog_path = os.path.join(realdata_path, latest_segment_folder, 'qlog.zst')
    
    if not os.path.exists(qlog_path):
        return jsonify({"error": f"qlog.zst not found in the latest drive log: {qlog_path}"}), 404

    script_path = '/data/openpilot/openpilot/selfdrive/debug/fingerprint_from_route.py'
    command = ['python', script_path, qlog_path]
    
    print(f"Executing command: {' '.join(command)}")

    try:
        result = subprocess.run(command, capture_output=True, text=True, check=True)
        full_output = result.stdout
        
        lines = full_output.splitlines()
        
        start_index = -1
        for i, line in enumerate(lines):
            if "FW fingerprint:" in line:
                start_index = i + 1
                break
        
        if start_index == -1:
            return jsonify({"error": "Could not find 'FW fingerprint:' in the script output.", "output": full_output}), 500

        end_index = -1
        for i, line in enumerate(lines[start_index:], start=start_index):
            if "VIN:" in line:
                end_index = i
                break

        if end_index == -1:
            extracted_lines = lines[start_index:]
        else:
            extracted_lines = lines[start_index:end_index]
        
        final_result = "\n".join([line for line in extracted_lines if line.strip()])

        return jsonify({"fingerprint": final_result})

    except subprocess.CalledProcessError as e:
        error_message = f"Script execution failed (Exit Code: {e.returncode}):\n{e.stderr}"
        print(error_message)
        return jsonify({"error": error_message}), 500
    except Exception as e:
        print(f"An unexpected error occurred: {e}")
        return jsonify({"error": str(e)}), 500


@app.route("/cmd/terminal_start", methods=["POST"])
def terminal_start():
    cmd_uuid = str(uuid.uuid4())
    print(f"[DEBUG] Terminal started. UUID={cmd_uuid}")

    def pty_runner():
        try:
            pid, master_fd = pty.fork()
        except OSError as e:
            print(f"[ERROR] pty.fork failed: {e}")
            return

        if pid == 0:
            try:
                os.chdir("/data/openpilot")

                os.environ["TERM"] = "xterm-256color"
                os.environ["PS1"] = (
                    "\033[01;32m\\u@\\h\033[00m:"
                    "\033[01;34m\\w\033[00m\\$ "
                )

                os.execv(
                    "/bin/bash",
                    ["/bin/bash", "-l", "-i"]
                )

            except Exception as e:
                with open("/data/log/kisa_agent_terminal_error.log", "a") as f:
                    f.write(f"CRITICAL: bash exec failed: {e}\n")
                os._exit(1)

        else:
            try:
                fcntl.ioctl(master_fd, termios.TIOCSWINSZ, struct.pack("HHHH", 32, 100, 0, 0))
            except OSError:
                pass
            decoder = codecs.getincrementaldecoder("utf-8")(errors="replace")
            PTY_SESSIONS[cmd_uuid] = {
                "pid": pid,
                "fd": master_fd,
                "output": "",
                "lock": threading.Lock()
            }
            try:
                while True:
                    r, _, _ = select.select([master_fd], [], [], 1.0)
                    if not r:
                        try:
                            if os.waitpid(pid, os.WNOHANG)[0] != 0:
                                break
                        except OSError:
                            break
                        continue

                    try:
                        data = os.read(master_fd, 4096)
                        if not data:
                            break
                        if cmd_uuid in PTY_SESSIONS:
                            PTY_SESSIONS[cmd_uuid]["output"] += decoder.decode(data)
                        else:
                            break
                    except OSError:
                        break
            finally:
                print(f"[DEBUG] Cleaning up session {cmd_uuid}")
                if cmd_uuid in PTY_SESSIONS:
                    try:
                        os.close(master_fd)
                    except OSError:
                        pass
                    del PTY_SESSIONS[cmd_uuid]

    threading.Thread(target=pty_runner, daemon=True).start()

    return jsonify({
        "status": "started",
        "id": cmd_uuid
    })


@app.route("/cmd/terminal_write", methods=["POST"])
def terminal_write():
    data = request.get_json(silent=True) or {}
    cmd_uuid = data.get("id")
    input_str  = data.get("input", "")

    session = PTY_SESSIONS.get(cmd_uuid)
    if not session:
        return jsonify({"error": "invalid session id"}), 404

    try:
        os.write(session["fd"], input_str.encode())
        return jsonify({"status": "ok"})
    except (OSError, IOError) as e:
        return jsonify({"error": str(e)}), 500

@app.route("/cmd/terminal_read", methods=["GET"])
def terminal_read():
    cmd_uuid = request.args.get("id")

    session = PTY_SESSIONS.get(cmd_uuid)
    if not session:
        return jsonify({
            "output": "\n[Session closed]\n",
            "done": True
        })

    with session["lock"]:
        output = session["output"]
        session["output"] = ""

    return jsonify({
        "output": output,
        "done": False
    })

@app.route("/cmd/terminal_close", methods=["POST"])
def terminal_close():
    data = request.get_json(silent=True) or {}
    cmd_uuid = data.get("id")

    if not cmd_uuid or cmd_uuid not in PTY_SESSIONS:
        return jsonify({"error": "session not found"}), 404

    session = PTY_SESSIONS[cmd_uuid]
    pid = session.get("pid")

    print(f"[DEBUG] Cleaning up session {cmd_uuid}")

    if pid:
        try:
            os.kill(pid, 15)
        except ProcessLookupError:
            pass
        except OSError as e:
            print(f"[ERROR] Failed to kill process {pid}: {e}")

    if cmd_uuid in PTY_SESSIONS:
        del PTY_SESSIONS[cmd_uuid]

    return jsonify({"status": "closed"})


# =========================
# UDP 브로드캐스트
# =========================
_cached_broadcasts = []
_cached_time = 0
CACHE_TTL = 10  # seconds
def get_broadcast_addresses():
    if not netifaces:
        return []

    global _cached_broadcasts, _cached_time

    now = time.time()
    if _cached_broadcasts and (now - _cached_time < CACHE_TTL):
        return _cached_broadcasts

    results = []

    for iface in netifaces.interfaces():
        if iface == "lo":
            continue

        iface_addrs = netifaces.ifaddresses(iface)
        if netifaces.AF_INET not in iface_addrs:
            continue

        for link in iface_addrs[netifaces.AF_INET]:
            ip = link.get("addr")
            netmask = link.get("netmask")

            if not ip or not netmask:
                continue

            try:
                network = ipaddress.IPv4Network(f"{ip}/{netmask}", strict=False)
                results.append((str(network.broadcast_address), ip))
            except Exception:
                pass

    _cached_broadcasts = results
    _cached_time = now

    return results

def udp_broadcast_loop():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

    while True:
        if not is_client_alive():
            for bcast, local_ip in get_broadcast_addresses():
                msg = json.dumps({
                    "device": "kisapilot",
                    "ip": local_ip,
                    "port": HTTP_PORT
                }).encode("utf-8")

                try:
                    sock.sendto(msg, (bcast, UDP_PORT))
                    # print(
                    #     f"UDP Broadcasting: {bcast}:{UDP_PORT} "
                    #     f"msg={msg.decode('utf-8')}"
                    # )
                except Exception as e:
                    print("UDP error:", e)

        time.sleep(UDP_INTERVAL)

# =========================
# Main
# =========================
def main():
    t = threading.Thread(target=udp_broadcast_loop, daemon=True)
    t.start()

    app.run(host="0.0.0.0", port=HTTP_PORT, threaded=True)


if __name__ == "__main__":
    main()
