from gem5.resources.resource import obtain_resource

resource = obtain_resource("")

print(f"the resources is available at {resource.get_local_path()}")