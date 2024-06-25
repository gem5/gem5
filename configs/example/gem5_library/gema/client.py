# configs/example/gem5_library/gema/client.py
# Example client to interact with gEMA host.

import json
import os

import requests


class APIClient:
    def __init__(self, base_url):
        self.base_url = base_url

    def get_help(self):
        endpoint = "/help"
        response = requests.get(self.base_url + endpoint)
        return self._handle_response(response)

    def get_config(self):
        endpoint = "/config/options"
        response = requests.get(self.base_url + endpoint)
        return self._handle_response(response)

    def get_saved(self):
        endpoint = "/simulation/saved"
        response = requests.get(self.base_url + endpoint)
        return self._handle_response(response)

    def configure_simulation(self, config_id, config_data):
        endpoint = f"/simulation/{config_id}/configure"
        response = requests.put(self.base_url + endpoint, json=config_data)
        return self._handle_response(response)

    def run_simulation(self, config_id):
        endpoint = f"/simulation/{config_id}/run"
        response = requests.put(self.base_url + endpoint)
        return self._handle_response(response)

    def shutdown_server(self):
        endpoint = "/shutdown"
        response = requests.put(self.base_url + endpoint)
        return self._handle_response(response)

    def _handle_response(self, response):
        if response.status_code == 404:
            return "Error: Endpoint not found"
        if response.status_code == 400:
            return f"Bad Request: {response.text}"
        if response.status_code == 500:
            return f"Server Error: {response.text}"
        return response.text

    def check_port(self):
        try:
            response = requests.get(self.base_url)
            if response.status_code == 200 or response.status_code == 404:
                return True
        except requests.exceptions.ConnectionError:
            return False
        return False


if __name__ == "__main__":
    port = input("Please enter the port number: ")
    base_url = f"http://localhost:{port}"
    client = APIClient(base_url)

    if not client.check_port():
        print(
            f"Error: Cannot connect to port {port}. Please ensure the server is running and the port is correct."
        )
    else:
        while True:
            print("\nSelect an endpoint to interact with:")
            print("1. GET /help")
            print("2. GET /config/options")
            print("3. GET /simulation/saved")
            print("4. PUT /simulation/{config_id}/configure")
            print("5. PUT /simulation/{config_id}/run")
            print("6. PUT /shutdown")
            print("7. Exit")

            choice = input("Enter the number of your choice: ")

            match choice:
                case "1":
                    print("Help Endpoint Response:")
                    response = client.get_help()
                    print(response)

                case "2":
                    print("Config Endpoint Response:")
                    response = client.get_config()
                    print(response)

                case "3":
                    print("Saved Endpoint Response:")
                    response = client.get_saved()
                    print(response)

                case "4":
                    config_id = input("Enter configuration ID: ")
                    json_path = input(
                        "Enter the path to the JSON configuration file: "
                    )

                    if os.path.exists(json_path):
                        with open(json_path) as file:
                            config_data = json.load(file)
                        print("Configure Simulation Response:")
                        response = client.configure_simulation(
                            config_id, config_data
                        )
                        print(response)
                    else:
                        print(f"File not found: {json_path}")

                case "5":
                    config_id = input("Enter configuration ID: ")
                    print("Run Simulation Response:")
                    response = client.run_simulation(config_id)
                    print(response)

                case "6":
                    print("Shutdown Server Response:")
                    response = client.shutdown_server()
                    print(response)

                case "7":
                    print("Exiting program.")
                    break

                case _:
                    print("Invalid choice, please try again.")
