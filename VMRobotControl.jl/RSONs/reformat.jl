# This is a script to reformat the RSON files from a prerelease version to 
# "v0.1.0". This script will reorder the keys in the JSON file to match the
# order in the key_order array, recursively. 
using JSON
using OrderedCollections

key_order = [
    "rson_version",
    "type",
    "name",
    "robot",
    "virtual_mechanism",
    "frames",
    "joints",
    "coordinates",
    "components",
    "materials",
    "visuals",

    "frame",
    "point",
    
    "parent", 
    "child",
    "axis",

    "origin",
    "rotation",

    "material",
    "length",
    "radius",
    "size"
]

function rewrite_named_lists_to_dicts(val)
    val
end

function item_with_name_to_pair(item::AbstractDict)
    name = item["name"] 
    data = OrderedDict(key => val for (key, val) in item if key != "name")
    return name => rewrite_named_lists_to_dicts(data)
end

function rewrite_named_lists_to_dicts(list::Vector)
    if all(isa.(list, Dict))
        ret = OrderedDict{String, Any}(
            item_with_name_to_pair(item) for item in list
        )
        return ret
    else
        return list
    end
end

function rewrite_named_lists_to_dicts(dict::AbstractDict)
    # Establish order of keys
    _keys = String[]
    append!(_keys, key_order)
    for key in keys(dict)
        !(key in _keys) && push!(_keys, key)
    end

    return OrderedDict(
        key => rewrite_named_lists_to_dicts(dict[key])
        for key in _keys if haskey(dict, key)
    )
end

for (root, dirs, files) in walkdir(joinpath(@__DIR__, "rsons"))
# for (root, dirs, files) in zip([".\\RSONs\\rsons"], [""], [["all_coords.rson"]])
    for file in files
        if endswith(".rson")(file)
            println(joinpath(root, file)) # path to files
            new_data = open(joinpath(root, file); read=true) do f
                data = JSON.parsefile(joinpath(root, file); use_mmap=false)
                for key in keys(data)
                    @assert key in key_order "Key $key from $file not in key_order"
                end
                if !haskey(data, "rson_version")
                    data["rson_version"] = "0.1.0"
                end
                new_data = rewrite_named_lists_to_dicts(data)
                new_data
            end
            # JSON.print(new_data, 4) # Indent 4 spaces
            open(joinpath(root, file); write=true) do f
                JSON.print(f, new_data, 4) # Indent 4 spaces
            end
        end
    end
end