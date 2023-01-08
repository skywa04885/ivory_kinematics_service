use ivory_bus::messages::Message;

const INVERSE_KINEMATICS_IK_EPSILON: f64 = 0.01;

const LEG_LENGTH_0: f64 = 0.0;
const LEG_LENGTH_1: f64 = 25.0;
const LEG_LENGTH_2: f64 = 25.0;

mod config;

fn build_torso() -> ivory_kinematics::Torso {
    ivory_kinematics::Torso::builder().build()
}

fn build_legs() -> [ivory_kinematics::Leg; 4] {
    return [
        ivory_kinematics::Leg::builder(0)
            .thetas(nalgebra::Vector3::new(
                0.0,
                f64::to_radians(-45.0),
                f64::to_radians(90.0),
            ))
            .lengths(nalgebra::Vector3::<f64>::new(
                LEG_LENGTH_0,
                LEG_LENGTH_1,
                LEG_LENGTH_2,
            ))
            .build(),
        ivory_kinematics::model::leg::Leg::builder(1)
            .thetas(nalgebra::Vector3::new(
                0.0,
                f64::to_radians(-45.0),
                f64::to_radians(90.0),
            ))
            .lengths(nalgebra::Vector3::<f64>::new(
                LEG_LENGTH_0,
                LEG_LENGTH_1,
                LEG_LENGTH_2,
            ))
            .build(),
        ivory_kinematics::model::leg::Leg::builder(2)
            .thetas(nalgebra::Vector3::new(
                0.0,
                f64::to_radians(-45.0),
                f64::to_radians(90.0),
            ))
            .lengths(nalgebra::Vector3::<f64>::new(
                LEG_LENGTH_0,
                LEG_LENGTH_1,
                LEG_LENGTH_2,
            ))
            .build(),
        ivory_kinematics::model::leg::Leg::builder(3)
            .thetas(nalgebra::Vector3::new(
                0.0,
                f64::to_radians(-45.0),
                f64::to_radians(90.0),
            ))
            .lengths(nalgebra::Vector3::<f64>::new(
                LEG_LENGTH_0,
                LEG_LENGTH_1,
                LEG_LENGTH_2,
            ))
            .build(),
    ];
}

fn send_pose_change_update(
    bus: std::sync::Arc<std::sync::Mutex<ivory_bus::Bus>>,
    kinematic_solver: &ivory_kinematics::Solver,
) -> Result<(), ivory_kinematics::Error> {
    let message = ivory_bus::messages::cpanel::updates::PoseChange::builder([
        kinematic_solver.fk_vertices_for_leg(0)?,
        kinematic_solver.fk_vertices_for_leg(1)?,
        kinematic_solver.fk_vertices_for_leg(2)?,
        kinematic_solver.fk_vertices_for_leg(3)?,
    ])
    .build();

    bus.lock().unwrap().publish_message(message);

    Ok(())
}

fn main() {
    let torso: ivory_kinematics::Torso = build_torso();
    let legs: [ivory_kinematics::Leg; 4] = build_legs();

    let mut kinematic_solver: ivory_kinematics::Solver =
        ivory_kinematics::Solver::builder(torso, legs)
            .pseudo_inverse_epsilon(0.001)
            .build();

    let (instruction_sender, mut instruction_receiver) =
        tokio::sync::mpsc::channel::<(std::borrow::Cow<'static, str>, Vec<u8>)>(80);

    let config: config::Config = config::Config::from_file("./env/config.toml".into()).unwrap();

    let bus = std::sync::Arc::new(std::sync::Mutex::new(ivory_bus::Bus::new(
        config.rabbitmq.uri.into(),
        config.rabbitmq.exchange.into(),
        vec![(
            "kinematics_instructions".into(),
            "kinematics.instructions.*".into(),
            std::sync::Arc::new(instruction_sender),
        )],
    )));

    log::set_boxed_logger(std::boxed::Box::new(ivory_bus::logger::Simple::new(
        bus.clone(),
        "kinematics_service".into(),
    )))
    .map(|()| log::set_max_level(log::LevelFilter::Trace))
    .unwrap();

    log::info!("Kinematics service has started.");

    while let Some((routing_key, buffer)) = instruction_receiver.blocking_recv() {
        let (_, instruction) = match routing_key.rsplit_once('.') {
            Some(segs) => segs,
            None => {
                log::error!("Failed to parse routing key: {}", routing_key);
                continue;
            }
        };

        match instruction {
            "change_torso_orientation" => {
                let message =
                    ivory_bus::messages::kinematics::instructions::ChangeTorsoOrientation::decode(
                        &buffer,
                    );

                log::debug!("Changin torso orientation to {:?}", message.orientation);

                let ik_result: Result<(), ivory_kinematics::Error> = if message.relative {
                    kinematic_solver.orient_torso_absolute(
                        &message.orientation,
                        Some(INVERSE_KINEMATICS_IK_EPSILON),
                    )
                } else {
                    kinematic_solver.orient_torso_absolute(
                        &message.orientation,
                        Some(INVERSE_KINEMATICS_IK_EPSILON),
                    )
                };

                if let Err(error) = ik_result {
                    log::error!("Failed to perform inverse kinematics solving: {:?}", error);
                    continue;
                }

                send_pose_change_update(bus.clone(), &kinematic_solver).unwrap();
            }
            "change_torso_position" => {
                let message =
                    ivory_bus::messages::kinematics::instructions::ChangeTorsoPosition::decode(
                        &buffer,
                    );

                log::debug!("Changin torso position to {:?}", message.position);

                let ik_result: Result<(), ivory_kinematics::Error> = if message.relative {
                    kinematic_solver
                        .move_torso_relative(&message.position, Some(INVERSE_KINEMATICS_IK_EPSILON))
                } else {
                    kinematic_solver
                        .move_torso_absolute(&message.position, Some(INVERSE_KINEMATICS_IK_EPSILON))
                };

                if let Err(error) = ik_result {
                    log::error!("Failed to perform inverse kinematics solving: {:?}", error);
                    continue;
                }

                send_pose_change_update(bus.clone(), &kinematic_solver).unwrap();
            }
            "change_paw_position" => {
                let message =
                    ivory_bus::messages::kinematics::instructions::ChangePawPosition::decode(
                        &buffer,
                    );

                log::debug!(
                    "Moving paw {} towards {} position {:?}",
                    message.leg,
                    if message.relative {
                        "relative"
                    } else {
                        "absolute"
                    },
                    message.position
                );

                let ik_result: Result<(), ivory_kinematics::Error> = if message.relative {
                    kinematic_solver.move_paw_relative(
                        message.leg,
                        &message.position,
                        Some(INVERSE_KINEMATICS_IK_EPSILON),
                    )
                } else {
                    kinematic_solver.move_paw_absolute(
                        message.leg,
                        &message.position,
                        Some(INVERSE_KINEMATICS_IK_EPSILON),
                    )
                };

                if let Err(error) = ik_result {
                    log::error!("Failed to perform inverse kinematics solving: {:?}", error);
                    continue;
                }

                send_pose_change_update(bus.clone(), &kinematic_solver).unwrap();
            }
            instruction => log::warn!("Unrecognized instruction: \"{}\"", instruction),
        }
    }
}
