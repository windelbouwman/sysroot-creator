//! Idea is to download and extract development files from
//! ubuntu ports to assist in cross compilation.
//!

struct SysrootSettings {
    sources: Vec<PackageSource>,
    base_folder: std::path::PathBuf,
    distribution_version: String,
    architecture: String,
    packages: Vec<String>,
}

struct PackageSource {
    name: String,
    base_url: String,
    sections: Vec<String>,
}

fn main() {
    let matches = clap::App::new("sysroot-creator")
        .author("Windel Bouwman")
        .arg(
            clap::Arg::with_name("v")
                .short("v")
                .multiple(true)
                .help("Specify the level of verbosity"),
        )
        .arg(
            clap::Arg::with_name("cfg")
                .help("The configuration file for the sysroot")
                .required(true)
                .index(1),
        )
        .get_matches();

    let verbosity = matches.occurrences_of("v");

    let log_level = if verbosity > 1 {
        log::LevelFilter::Trace
    } else if verbosity > 0 {
        log::LevelFilter::Debug
    } else {
        log::LevelFilter::Info
    };

    let settings_file_path = std::path::Path::new(matches.value_of("cfg").unwrap());

    simple_logger::SimpleLogger::new()
        .with_level(log_level)
        .init()
        .unwrap();

    // Step 0: determine settings:
    let settings = prepare_settings(&settings_file_path);

    // Create base folder:
    log::info!("Outputting to: {}", settings.base_folder.display());
    if !settings.base_folder.is_dir() {
        log::info!("Creating folder: {}", settings.base_folder.display());
        std::fs::create_dir(&settings.base_folder).expect("This should work");
    }

    // Step 1: download database files:
    let package_database = apt_update(&settings);

    // Step 2: resolve dependencies:
    let packages_to_install = resolve_deps(&package_database, &settings.packages);

    // Step 3: download and extract selected pacakges
    extract_packages_to_folder(&packages_to_install, &settings.base_folder);
}

/// Roughly perform 'apt update'
fn apt_update(settings: &SysrootSettings) -> PackageDb {
    let mut package_database: PackageDb = Default::default();
    for source in &settings.sources {
        for section in &source.sections {
            download_package_database(
                &source.name,
                &source.base_url,
                &settings.distribution_version,
                &settings.architecture,
                section,
                &settings.base_folder,
                &mut package_database,
            );
        }
    }
    package_database
}

#[derive(serde::Deserialize)]
struct ConfigFile {
    sources: std::collections::HashMap<String, ConfigFileSource>,
    distribution_version: String,
    architecture: String,
    folder: String,
    packages: Vec<String>,
}

#[derive(serde::Deserialize)]
struct ConfigFileSource {
    url: String,
    sections: Vec<String>,
}

fn prepare_settings(settings_file_path: &std::path::Path) -> SysrootSettings {
    let txt = std::fs::read_to_string(settings_file_path).unwrap();
    let cfg_file: ConfigFile = toml::from_str(&txt).unwrap();

    // let cache_folder: String = "bla".to_owned();
    // TODO: display extremely friendly errors during parsing phase.
    let distribution_version: String = cfg_file.distribution_version;
    let architecture: String = cfg_file.architecture;
    let packages: Vec<String> = cfg_file.packages;
    let base_folder = std::path::PathBuf::from(&cfg_file.folder);
    let sources: Vec<PackageSource> = cfg_file
        .sources
        .iter()
        .map(|(key, val)| PackageSource {
            name: key.clone(),
            base_url: val.url.clone(),
            sections: val.sections.clone(),
        })
        .collect();

    SysrootSettings {
        sources,
        base_folder,
        distribution_version,
        architecture,
        packages,
    }
}

type PackageDb = std::collections::HashMap<String, Vec<PackageDescription>>;

/// This roughly corresponds to `apt update`
fn download_package_database(
    source_name: &str,
    base_url: &str,
    distribution: &str,
    architecture: &str,
    section: &str,
    base_folder: &std::path::Path,
    db: &mut PackageDb,
) {
    let distribution_packages_url = url::Url::parse(&format!(
        "{}/dists/{}/{}/binary-{}/Packages.gz",
        base_url, distribution, section, architecture,
    ))
    .unwrap();

    let domain = distribution_packages_url.domain().unwrap();
    let packages_gz_filename = format!(
        "{}-{}-{}-{}-{}-Packages.gz",
        source_name, domain, distribution, section, architecture
    );
    let packages_gz = base_folder.join(std::path::Path::new(&packages_gz_filename));
    download_file(&distribution_packages_url, &packages_gz);

    // Process the gzipped file:
    let f = std::fs::File::open(&packages_gz).unwrap();
    use flate2::read::GzDecoder;
    let f = GzDecoder::new(f);
    let packages = parse_packagez(base_url, f);

    // contrapt lookup table:
    inject_into_database(packages, db);
}

fn inject_into_database(packages: Vec<PackageDescription>, database: &mut PackageDb) {
    for package in packages {
        if database.contains_key(&package.name) {
            // log::error!("duplicate package! {}: ", package.name);
            let candidates = database.get_mut(&package.name).unwrap();
            // if !candidates.iter().any(|p| p == &package) {
            candidates.push(package);
            // }
            // TODO: sort by version?
            // candidates.sort_by_key();
            // println!("{:?} <> {:?}", package, );
        } else {
            database.insert(package.name.clone(), vec![package]);
        }
    }
}

/// Given a list of packages, resolve dependencies
fn resolve_deps(package_database: &PackageDb, packages: &[String]) -> Vec<PackageDescription> {
    let mut to_install_packages = vec![];
    let mut added = std::collections::HashSet::<String>::new();
    let mut worklist: Vec<String> = vec![];

    for package_name in packages {
        added.insert(package_name.clone());
        worklist.push(package_name.clone());
    }

    while let Some(package_name) = worklist.pop() {
        if let Some(package_candidates) = package_database.get(&package_name) {
            // TODO: get best suitable package based on version requirements.
            // Get first package
            let package: PackageDescription = if package_candidates.len() > 1 {
                let mut sorted_package_candidates: Vec<PackageDescription> =
                    (*package_candidates).clone();
                // TODO: take into account version requirements
                sorted_package_candidates
                    .sort_by(|a, b| deb_version::compare_versions(&a.version, &b.version));
                sorted_package_candidates.reverse();
                // println!("Secting from:");
                // for p in &sorted_package_candidates {
                //     println!("- {}: {} ({})", p.name, p.version, p.md5sum);
                // }
                sorted_package_candidates.get(0).unwrap().clone()
            } else {
                package_candidates.get(0).unwrap().clone()
            };

            log::debug!(
                "Marking for installation: {} -> Depends upon: {:?}",
                package_name,
                package.depends
            );
            for dep in &package.depends {
                let dep_name: String = match &dep {
                    PackageDependency::Plain(name) => name.clone(),
                    // PackageDependency::Versioned((name, _version)) => name.clone(),
                    PackageDependency::OneOf(options) => {
                        match &options[0] {
                            PackageDependency::Plain(name) => name.clone(),
                            // PackageDependency::Versioned((name, _version)) => name.clone(),
                            PackageDependency::OneOf(_) => {
                                unreachable!(
                                    "This must not occur (nested alternative dependencies)!"
                                );
                            }
                        }
                    }
                };
                if !added.contains(&dep_name) {
                    worklist.push(dep_name.clone());
                    added.insert(dep_name);
                }
            }
            to_install_packages.push(package);
        } else {
            log::error!("Package not found: {} (hoping for the best)", package_name);
        }
    }
    to_install_packages
}

fn extract_packages_to_folder(packages: &[PackageDescription], base_folder: &std::path::Path) {
    let output_folder: std::path::PathBuf = base_folder.join(std::path::Path::new("sysroot"));
    let package_folder: std::path::PathBuf =
        base_folder.join(std::path::Path::new("packages-folder"));

    // Create sysroot folder:
    if !output_folder.is_dir() {
        log::info!("Creating output folder: {}", output_folder.display());
        std::fs::create_dir(&output_folder).unwrap();
    }

    // Create packages folder:
    if !package_folder.is_dir() {
        log::info!("Creating package folder: {}", package_folder.display());
        std::fs::create_dir(&package_folder).expect("This should work");
    }

    // TODO: progress bar?
    for (index, package) in packages.iter().enumerate() {
        log::info!(
            "({}/{}) Processing {}",
            index + 1,
            packages.len(),
            package.name
        );

        let deb_filename =
            std::path::Path::new(package.url.path_segments().unwrap().last().unwrap());
        let deb_path = package_folder.join(deb_filename);
        download_file(&package.url, &deb_path);
        check_md5(&deb_path, &package.md5sum);
        process_deb(&deb_path, &output_folder);
    }
}

/// Check if the md5 sum of a file is correct
fn check_md5(filename: &std::path::Path, md5sum: &str) {
    let mut f = std::fs::File::open(filename).unwrap();
    let mut context = md5::Context::new();
    std::io::copy(&mut f, &mut context).unwrap();
    let file_md5_digest = format!("{:x}", context.compute());
    log::debug!(
        "MD5 checksum of {}: {}",
        filename.display(),
        file_md5_digest
    );
    if file_md5_digest == md5sum {
        log::trace!("MD5 checksum is ok.");
    } else {
        log::error!("MD5 checksum mismatch: {} != {}", file_md5_digest, md5sum);
        // TBD: quit? What now?
    }
}

/// Extract a debian package
///
/// A deb archive is an `ar` file with
/// a `data.tar.xz` and `control.tar.xz` section.
fn process_deb(filename: &std::path::Path, output_folder: &std::path::Path) {
    log::debug!("Extracting {}", filename.display());
    let file = std::fs::File::open(filename).unwrap();

    let mut archive = ar::Archive::new(file);
    while let Some(entry_result) = archive.next_entry() {
        let entry = entry_result.unwrap();
        let name = std::str::from_utf8(entry.header().identifier()).unwrap();
        log::trace!("Entry! {}", name);
        // TBD: improve this hardcoded name?
        if name == "data.tar.xz" {
            extract_data_tar(entry, output_folder);
        }
    }
}

/// process data.tar.xz
fn extract_data_tar<R>(entry: R, output_folder: &std::path::Path)
where
    R: std::io::Read,
{
    let uncompressor = xz2::read::XzDecoder::new(entry);
    let mut archive2 = tar::Archive::new(uncompressor);

    let abs_destination_path = output_folder.canonicalize().unwrap();

    for data_file in archive2.entries().unwrap() {
        let mut data_file = data_file.unwrap();
        // println!("Entry type: {:?}", data_file.header().entry_type());
        // Only unpack regular files!

        // Use entry.path to ensure we support long filenames:
        let p = data_file.path().unwrap();
        let output_path = glue_paths(output_folder, &p);
        log::trace!("Processing tar entry: {}", p.display());
        if p.is_relative() {
            match data_file.header().entry_type() {
                tar::EntryType::Regular => {
                    if output_path.exists() {
                        log::debug!("Skipping existing file: {}", output_path.display());
                    } else {
                        log::debug!("Extracting: {}", output_path.display());
                        let mut out_file = std::fs::OpenOptions::new()
                            .write(true)
                            .create_new(true)
                            .open(output_path)
                            .unwrap();
                        std::io::copy(&mut data_file, &mut out_file).unwrap();
                    }
                }
                tar::EntryType::Symlink => {
                    if path_exists_or_symlink(&output_path) {
                        log::debug!("Skipping existing symlink: {}", output_path.display());
                    } else {
                        let link_target = data_file.link_name().unwrap().unwrap();
                        // TODO: this fails on windows!

                        // If the link target is relative, then all is well
                        // otherwise we need to change the absolute path.
                        if link_target.is_relative() {
                            sym_link(&link_target, &output_path);
                        } else {
                            let new_abs_path = rebase_path(&abs_destination_path, &link_target);
                            log::debug!(
                                "Absolute symlink: {} --> {}",
                                output_path.display(),
                                new_abs_path.display()
                            );
                            sym_link(&new_abs_path, &output_path);
                        }
                    }
                }
                tar::EntryType::Directory => {
                    if output_path.exists() {
                        log::debug!("Skipping existing folder: {}", output_path.display());
                    } else {
                        log::debug!("Creating directory {}!", output_path.display());
                        std::fs::create_dir(output_path).unwrap();
                    }
                }
                _ => {
                    // ignore for now.
                    log::debug!("Ignoring: {}", p.display());
                }
            }
        } else {
            log::warn!("Skipping non-relative entry: {}", p.display());
        }
    }
}

fn path_exists_or_symlink(path: &std::path::Path) -> bool {
    std::fs::symlink_metadata(path).is_ok()
}

#[cfg(target_os = "windows")]
fn sym_link(link_target: &std::path::Path, output_path: &std::path::Path) {
    log::debug!(
        "Sym linking {} -> {}",
        output_path.display(),
        link_target.display()
    );
    if let Err(err) = std::os::windows::fs::symlink_file(link_target, output_path) {
        log::error!("Something failed during symlink: {}", err);
    }
}

#[cfg(target_os = "linux")]
fn sym_link(link_target: &std::path::Path, output_path: &std::path::Path) {
    log::debug!(
        "Sym linking {} -> {}",
        output_path.display(),
        link_target.display()
    );
    if let Err(err) = std::os::unix::fs::symlink(link_target, output_path) {
        log::error!("Something failed during symlink: {}", err);
    }
}

/// Attach a relative path to a base path
fn glue_paths(base_path: &std::path::Path, p: &std::path::Path) -> std::path::PathBuf {
    assert!(p.is_relative());
    let mut output_path: std::path::PathBuf = base_path.to_path_buf();
    // TODO: use canonicalize?
    // let output_path = output_folder.join(p).canonicalize().unwrap();

    let mut components = p.components();
    let first = components.next().unwrap();
    assert!(first == std::path::Component::CurDir);
    for x in components {
        // println!("Component: {:?}", x);
        match x {
            std::path::Component::Normal(x) => {
                output_path.push(x);
            }
            _ => {
                unimplemented!();
            }
        }
    }
    output_path
}

/// Rebase an absolute path onto another absolute path.
fn rebase_path(output_folder: &std::path::Path, p: &std::path::Path) -> std::path::PathBuf {
    assert!(p.is_absolute());
    assert!(output_folder.is_absolute());
    let mut result_path: std::path::PathBuf = output_folder.to_path_buf();

    let mut components = p.components();
    let first = components.next().unwrap();
    assert!(first == std::path::Component::RootDir);
    for x in components {
        // println!("Component: {:?}", x);
        match x {
            std::path::Component::Normal(x) => {
                result_path.push(x);
            }
            _ => {
                unimplemented!();
            }
        }
    }
    result_path
}

/// Download a file if it not already exists.
fn download_file(url: &url::Url, filename: &std::path::Path) {
    if filename.exists() {
        log::debug!(
            "File {} already present, not re-downloading!",
            filename.display()
        );
    } else {
        log::info!("Downloading: {}", url);

        let mut body = reqwest::blocking::get(url.clone()).unwrap();
        if !body.status().is_success() {
            panic!("Something went wrong fetching {}: {}", url, body.status());
        }

        let mut out = std::fs::File::create(filename).expect("Should work!");
        std::io::copy(&mut body, &mut out).expect("Should work");
    }
}

#[derive(Debug, PartialEq, Clone)]
struct PackageDescription {
    name: String,
    version: String,
    provides: Option<Vec<String>>,
    depends: Vec<PackageDependency>,
    url: url::Url,
    md5sum: String,
}

#[derive(Debug, PartialEq, Clone)]
enum PackageDependency {
    Plain(String),
    // TODO: Versioned((String, String)),
    OneOf(Vec<PackageDependency>),
}

/// Parse dependency string
///
/// Examples:
/// 'libc'
/// 'fastrtps | otherdds'
fn parse_package_dependency(desc: &str) -> PackageDependency {
    let desc = desc.trim();
    if desc.contains('|') {
        let options = desc.split('|').map(parse_package_dependency).collect();
        PackageDependency::OneOf(options)
    } else if desc.contains('(') {
        let name = desc.split('(').next().unwrap().trim().to_owned();
        // TODO: parse version properly.
        // let version = ?
        // (name, None)
        PackageDependency::Plain(name)
    } else {
        PackageDependency::Plain(desc.to_owned())
    }
}

fn parse_depends(deps: &str) -> Vec<PackageDependency> {
    deps.split(',').map(parse_package_dependency).collect()
}

impl PackageDescription {
    fn from_keys(base_url: &str, keys: &std::collections::HashMap<String, String>) -> Self {
        let name: String = keys.get("Package").unwrap().to_owned();
        let version: String = keys.get("Version").unwrap().to_owned();
        let provides: Option<Vec<String>> = keys
            .get("Provides")
            .map(|p| p.split(',').map(|n| n.trim().to_owned()).collect());
        if let Some(_x) = &provides {
            // println!("{} provides {:?}", name, x);
        }
        let depends: Vec<PackageDependency> = keys
            .get("Depends")
            .map(|d| parse_depends(d))
            .unwrap_or_default();

        let filename: String = keys.get("Filename").unwrap().to_owned();
        let package_url = format!("{}/{}", base_url, filename);
        let url = url::Url::parse(&package_url).unwrap();

        let md5sum: String = keys.get("MD5sum").unwrap().to_owned();

        Self {
            name,
            version,
            provides,
            depends,
            url,
            md5sum,
        }
    }
}

/// Process the contents of a Packages.gz file into a list of package descriptions.
fn parse_packagez<R>(base_url: &str, mut f: R) -> Vec<PackageDescription>
where
    R: std::io::Read,
{
    let mut buf = String::new();
    f.read_to_string(&mut buf).unwrap();
    let paragraphs = debcontrol::parse_str(&buf).unwrap();

    let mut packages: Vec<PackageDescription> = vec![];
    for paragraph in paragraphs {
        let keys = paragraph_to_keys(paragraph);
        packages.push(PackageDescription::from_keys(base_url, &keys));
    }

    log::info!("Loaded {} packages", packages.len());

    packages
}

fn paragraph_to_keys(
    paragraph: debcontrol::Paragraph,
) -> std::collections::HashMap<String, String> {
    let mut keys: std::collections::HashMap<String, String> = std::collections::HashMap::new();
    for field in paragraph.fields {
        // println!("{}: {}", field.name, field.value);
        let key: &str = &field.name;
        if keys.contains_key(key) {
            panic!("Duplicate key: {}", key);
        } else {
            keys.insert(key.to_owned(), field.value.to_owned());
        }
    }
    keys
}

#[cfg(test)]
mod tests {

    #[test]
    fn test_glue_paths() {
        use super::glue_paths;
        let p1 = std::path::Path::new("/home/joe");
        let p2 = std::path::Path::new("./foo/bar.txt");
        let p3 = glue_paths(p1, p2);
        assert_eq!(p3, std::path::Path::new("/home/joe/foo/bar.txt"));
    }

    #[test]
    fn test_rebase_path() {
        use super::rebase_path;
        let p1 = std::path::Path::new("/home/joe");
        let p2 = std::path::Path::new("/foo/bar.txt");
        let p3 = rebase_path(p1, p2);
        assert_eq!(p3, std::path::Path::new("/home/joe/foo/bar.txt"));
    }
}
