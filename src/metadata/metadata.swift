import Photos

func getMetadata(imageData: Data) -> NSDictionary? {
    if let imageSource = CGImageSourceCreateWithData(imageData as CFData, nil) {
      if let dict = CGImageSourceCopyPropertiesAtIndex(imageSource, 0, nil) {
        return dict
      }
    }
    return nil
}

func main() {
    guard CommandLine.argc > 1 else {
        print("Error: Provide an image path as argument.")
        return
    }
    let fileURL = URL(fileURLWithPath: CommandLine.arguments[1])    
    guard let data = try? Data(contentsOf: fileURL) else {
        print("Error: Could not get image data.")
        return
    }
    guard let metadata = getMetadata(imageData: data) else {
        print("Error: Could not get metadata.")
        return
    }
    guard let exif = metadata["{Exif}"] else {
        print("{}")
        return
    }
    guard let jsonData = try? JSONSerialization.data(withJSONObject: exif, options: [.prettyPrinted]) else {
        print("Error: Could not parse json.")
        return
    }
    guard let json = String(data: jsonData, encoding: .utf8) else {
        print("Error: Could stringify json.")
        return
    }
    print(json)
}

main()