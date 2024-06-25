import json
import os
from http.server import BaseHTTPRequestHandler
from http.server import HTTPServer as GemaHTTPServer


class GemaServer:
    """Server class for managing gEMA backend operations."""

    def __init__(self, root, port):
        """Initialize the gEMA server class."""
        self.root = root
        self.port = port

    def _create_handler(self):
        """Generates the server handler and passes the root."""
        return lambda *args, **kwargs: GemaHandler(self.root, *args, **kwargs)

    def run(self):
        """Runs the gEMA server."""
        server_address = ("", self.port)
        handler = self._create_handler()
        GemaHTTP = GemaHTTPServer(server_address, handler)
        print(f"Starting server on port {self.port}.")
        print(
            "For help, access /help on the server URL or consult the documentation."
        )
        GemaHTTP.serve_forever()


class GemaHandler(BaseHTTPRequestHandler):
    """HTTP request handler with routes defined for service operations."""

    def __init__(self, root, *args, **kwargs):
        """Initialize the gEMA handler class."""
        self.root = root
        super().__init__(*args, **kwargs)

    def _set_headers(self, status=200, content_type="application/json"):
        """Set HTTP response headers."""
        self.send_response(status)
        self.send_header("Content-Type", content_type)
        self.send_header("Connection", "close")
        self.end_headers()

    def _send_output(self, data, status=200):
        """Send JSON response."""
        self._set_headers(status)
        response = json.dumps(data, indent=4).encode()
        self.wfile.write(response)

    def _send_error(self, status, message):
        """Send JSON error response."""
        self._set_headers(status)
        response = json.dumps({"error": message}, indent=4).encode()
        self.wfile.write(response)

    def _not_found(self):
        """Send 404 Not Found response."""
        self._send_error(404, "Not Found")

    def do_GET(self):
        """Handle GET requests."""
        endpoints = {
            "/config/options": self.send_config_options,
            "/simulation/saved": self.send_saved_simulations,
            "/help": self.list_endpoints,
        }
        handler = endpoints.get(self.path, self._not_found)
        handler()

    def do_PUT(self):
        """Handle PUT requests."""
        path = self.path.split("/")

        if self.path == "/shutdown":
            self.handle_shutdown()
        elif len(path) == 4 and path[1] == "simulation":
            config_id = path[2]
            action = path[3]
            if action == "run":
                self.handle_run_simulator(config_id)
            elif action == "configure":
                self.handle_external_data(config_id)
            else:
                self._send_error(400, "Simulation subcommand invalid.")
        else:
            self._not_found()

    def send_config_options(self):
        """Send configuration options."""
        try:
            options = self.root.retriever.get_config_options()
            self._send_output(options)
        except Exception as e:
            self._send_error(500, f"Internal Server Error: {str(e)}")

    def send_saved_simulations(self):
        """Send saved simulations."""
        try:
            data = self.root.sims
            self._send_output(data)
        except Exception as e:
            self._send_error(500, f"Internal Server Error: {str(e)}")

    def list_endpoints(self):
        """List available endpoints."""
        endpoints = {
            "GET /help": "Displays available endpoints",
            "GET /config/options": "Get configuration options",
            "GET /simulation/saved": "Get saved simulations",
            "PUT /simulation/{config_id}/configure": "Submit user data for simulation configuration",
            "PUT /simulation/{config_id}/run": "Run the simulation",
            "PUT /shutdown": "Shutdown the server",
        }
        self._send_output(endpoints)

    def handle_run_simulator(self, config_id):
        """Handle running a simulation."""
        try:
            sim_id = self.root.manager.get_lowest_sim_id(config_id)
            self.root.manager.start_subprocess(sim_id, config_id)
            response_message = f"Starting simulation id: {sim_id} using configuration id: {config_id}"
            self._send_output(response_message)
        except Exception as e:
            self._send_error(500, f"Internal Server Error: {str(e)}")

    def handle_shutdown(self):
        """Handle server shutdown."""
        try:
            message = f"Terminating gEMA server process, pid: {os.getpid()}"
            self._send_output({"message": message})
            print(message)
            os._exit(0)
        except Exception as e:
            self._send_error(500, f"Internal Server Error: {str(e)}")

    def handle_external_data(self, config_id):
        """Handle external data for simulation configuration."""
        try:
            content_length = int(self.headers["Content-Length"])
            data = self.rfile.read(content_length)
            received_data = json.loads(data.decode("utf-8"))
            self.root.configurator.save_config(config_id, received_data)
            response_message = f"Configured gem5 object, config_id: {config_id}. Ready to Simulate!"
            self._send_output(response_message)
        except json.JSONDecodeError:
            self._send_error(400, "Invalid JSON data received.")
        except Exception as e:
            self._send_error(500, f"Internal Server Error: {str(e)}")
